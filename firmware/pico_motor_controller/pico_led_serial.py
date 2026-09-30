# LED on-time + 4-wheel direct-drive PWM, controlled over USB serial.
# Also reads 4x quadrature wheel encoders (PIO decode, no CPU involvement)
# and reports counts over serial at ~50 Hz.
#
# Commands (one per line):
#   NN                     -> LED on-time = NN * 10 ms, off 200 ms, repeat
#   fl,fr,bl,br            -> float 4-wheel line (-1..1 each), production
#                             format from the Orin cmd_vel_serial node.
#                             Sentinel 1 direct-drive wiring (2026-09-24,
#                             mecanum mixer bypassed):
#                             FR -> GP18, BR -> GP19, FL -> GP20, BL -> GP21,
#                             pulse = 1500 + value * 400 us
#                             (1100-1900 us, GoBilda 1x20A controller spec).
#
# Encoder report (printed ~50 Hz, one block of four lines):
#   FL <count>
#   FR <count>
#   BL <count>
#   BR <count>
#
# Safety layers:
#   1. Software failsafe: no valid motor command for 200 ms -> neutral.
#   2. Hardware watchdog (WDT_TIMEOUT_MS), armed just before the main loop.
#      If the loop stalls for any reason the Pico resets, its PWM pins
#      float, and the GoBilda controller's signal-loss failsafe should stop
#      the motors. Confirm that on the bench with the wheels off the ground.
#   3. Any uncaught exception in the main loop: all channels neutral,
#      traceback printed, then reset.
#
# Development escape hatches (the watchdog plus the Ctrl-C lockout make a
# bad build hard to interrupt):
#   - Jumper ESCAPE_PIN to GND at power-up: the script exits to the REPL
#     before touching the motors.
#   - For BOOT_WINDOW_MS after every reset, no watchdog is armed and Ctrl-C
#     still works. The LED is solid on in the window, or blinks fast if the
#     reset was a watchdog reset.
#   - Test new code with `mpremote run file.py` rather than as main.py
#     until it is stable.
#
# PWM: 50 Hz (20 ms period), RC-style 1000-2000 us, 1500 = neutral.
from machine import Pin, PWM, WDT
import machine
import micropython
import rp2
import select, sys, time

ESCAPE_PIN = 22          # jumper to GND at power-up -> exit to the REPL
BOOT_WINDOW_MS = 2000    # Ctrl-C-able window after reset; 0 disables it
WDT_TIMEOUT_MS = 500     # a print() blocked on a non-draining host can trip it

led = Pin(25, Pin.OUT)
on_ms = 100  # default LED on-time: 10 x 10ms

# -------------------- Boot: escape hatches --------------------
# Nothing below this point has touched the motors yet, so the PWM pins are
# floating and the controller sees no signal.

if Pin(ESCAPE_PIN, Pin.IN, Pin.PULL_UP).value() == 0:
    led.on()
    sys.exit()

try:
    _wdt_reset = machine.reset_cause() == machine.WDT_RESET
except AttributeError:
    _wdt_reset = False

_t0 = time.ticks_ms()
while time.ticks_diff(time.ticks_ms(), _t0) < BOOT_WINDOW_MS:
    led.value(((time.ticks_ms() // 100) & 1) if _wdt_reset else 1)
    time.sleep_ms(10)

# From here on a stray 0x03 (Ctrl-C) byte on stdin must not kill the script
# while motors are spinning; interrupts stay off until reset.
micropython.kbd_intr(-1)

# -------------------- 4-wheel direct-drive PWM --------------------

PWM_FREQ = 50
PWM_PERIOD_US = 1000000 // PWM_FREQ  # 20000 us
NEUTRAL_US = 1500
FAILSAFE_MS = 200

# Direct drive, mecanum mixer bypassed (2026-09-24).
CHANNEL_PINS = {
    "fl": 20,
    "fr": 18,
    "bl": 21,
    "br": 19,
}

pwm = {}
for ch, gpio in CHANNEL_PINS.items():
    pwm[ch] = PWM(Pin(gpio))
    pwm[ch].freq(PWM_FREQ)

def set_pulse(ch, us):
    duty = int(65535 * us / PWM_PERIOD_US)
    pwm[ch].duty_u16(duty)

def float_to_pulse(v):
    # Map -1..1 onto 1100-1900 us (GoBilda 1x20A controller PWM spec),
    # 0 = neutral 1500 us.
    v = max(-1.0, min(1.0, v))
    return NEUTRAL_US + int(v * 400)

def neutral_all():
    for ch in CHANNEL_PINS:
        set_pulse(ch, NEUTRAL_US)

# Start all channels at neutral.
neutral_all()

# -------------------- Quadrature encoders --------------------
#
# goBILDA 5203 (19.2:1) encoders: 537.7 counts per output-shaft
# revolution after 4x decode (7 CPR motor-shaft encoder x 4x edges x
# 19.2:1 = 537.6). Verified empirically 2026-09-24: 20 hand-spun wheel
# revolutions measured ~10,653 counts (532.7/rev, within 1% of spec).
#
# Decode: PIO, one state machine per encoder, running the canonical RP2040
# quadrature program (raspberrypi/pico-examples, pio/quadrature_encoder).
# Each pass of its loop samples both pins, updates the signed count in the
# Y register, and pushes Y to the RX FIFO without blocking. No interrupts,
# so encoder noise can add bogus counts but cannot starve the CPU.
#
# The program uses a computed jump (mov pc, isr) into a table at ABSOLUTE
# instruction addresses 0-15, so it must be loaded at offset 0 of the PIO
# block. Nothing in MicroPython enforces that, so we clear PIO0 first and
# keep this the only asm_pio program in the file.
#
# Sampling clock: the loop takes at most ~14 cycles, so the maximum
# countable step rate is ENC_SM_FREQ / 14 (~70k steps/s at 1 MHz), about
# 25x the ~2.8k steps/s of a 312 RPM motor. A slower clock makes narrow
# noise spikes less likely to be sampled. This is a crude filter, not a
# substitute for RC filtering and cable routing.

ENC_SM_FREQ = 1000000

@rp2.asm_pio(in_shiftdir=rp2.PIO.SHIFT_LEFT, out_shiftdir=rp2.PIO.SHIFT_RIGHT)
def quadrature_encoder():
    # Computed-jump table at addresses 0-15. ISR = (old << 2) | new, so the
    # ROW is the OLD pin state and the COLUMN is the NEW pin state, where
    # state = (B << 1) | A (in_base is pin A, which lands in bit 0).
    # Instruction order is part of the encoding: do not reorder.
    jmp("update")        # old 00, new 00: no change
    jmp("decrement")     #         new 01
    jmp("increment")     #         new 10
    jmp("update")        #         new 11: invalid double step, ignore

    jmp("increment")     # old 01, new 00
    jmp("update")        #         new 01: no change
    jmp("update")        #         new 10: invalid double step, ignore
    jmp("decrement")     #         new 11

    jmp("decrement")     # old 10, new 00
    jmp("update")        #         new 01: invalid double step, ignore
    jmp("update")        #         new 10: no change
    jmp("increment")     #         new 11

    jmp("update")        # old 11, new 00: invalid double step, ignore
    jmp("increment")     #         new 01
    label("decrement")
    jmp(y_dec, "update")  #        new 10: Y--, then fall into the loop

    wrap_target()
    label("update")
    mov(isr, y)          # old 11, new 11: no change (address 15 is the loop top)
    push(noblock)        # publish the count; drop it if the FIFO is full

    label("sample_pins")
    out(isr, 2)          # ISR = previous state (low 2 bits of OSR)
    in_(pins, 2)         # ISR = (previous << 2) | current
    mov(osr, isr)        # remember it; its low 2 bits are the next "previous"
    mov(pc, isr)         # computed jump into the table

    label("increment")
    mov(y, invert(y))    # PIO has no add: Y = ~(~Y - 1) = Y + 1
    jmp(y_dec, "increment_cont")
    label("increment_cont")
    mov(y, invert(y))
    wrap()

# Set to -1 to flip one encoder's sign without touching wiring. The PIO
# state bit order differs from the old IRQ decoder, so verify the sign by
# pushing each wheel forward and flip here if it counts down.
FL_ENCODER_DIRECTION = 1
FR_ENCODER_DIRECTION = 1
BL_ENCODER_DIRECTION = 1
BR_ENCODER_DIRECTION = 1

# name, A gpio, B gpio, direction. Pins per spec:
# FR A/B = GP6/7, BR A/B = GP8/9, FL A/B = GP10/11, BL A/B = GP12/13.
# B must be A + 1: the PIO program reads two consecutive pins from A.
_ENCODER_DEFS = (
    ('FL', 10, 11, FL_ENCODER_DIRECTION),
    ('FR', 6, 7, FR_ENCODER_DIRECTION),
    ('BL', 12, 13, BL_ENCODER_DIRECTION),
    ('BR', 8, 9, BR_ENCODER_DIRECTION),
)

REPORT_MS = 20  # encoder report rate (50 Hz)

# Make our program the first one loaded on PIO0 so it lands at offset 0.
rp2.PIO(0).remove_program()

_ENC_COUNTS = [0, 0, 0, 0]
_ENC_DIRS = []
_SMS = []

for i, (_name, _a_gp, _b_gp, _dir) in enumerate(_ENCODER_DEFS):
    _a_pin = Pin(_a_gp, Pin.IN, Pin.PULL_UP)
    _b_pin = Pin(_b_gp, Pin.IN, Pin.PULL_UP)
    _ENC_DIRS.append(_dir)
    _sm = rp2.StateMachine(i, quadrature_encoder, freq=ENC_SM_FREQ,
                           in_base=_a_pin)
    # Start from a known state instead of leftover register contents:
    # OSR = the actual pin state (so the first sample cannot invent a
    # step), Y = 0 (counts start at zero).
    _sm.put((_b_pin.value() << 1) | _a_pin.value())
    _sm.exec("pull()")
    _sm.exec("set(y, 0)")
    _sm.active(1)
    _SMS.append(_sm)


def poll_encoders():
    # Read each state machine's current count. The PIO pushes every loop
    # pass (microseconds apart) and the RX FIFO is only 4 deep, so NEVER
    # loop "while the FIFO is not empty": it refills faster than Python can
    # drain it. Instead drain what is queued (those entries can be stale)
    # plus one more, which waits for a fresh sample. get() only blocks for
    # more than a few microseconds if the state machine has stopped, and
    # then the watchdog resets the Pico.
    for i, sm in enumerate(_SMS):
        for _ in range(sm.rx_fifo() + 1):
            val = sm.get()
        if val & 0x80000000:
            val -= 0x100000000
        _ENC_COUNTS[i] = val * _ENC_DIRS[i]


_last_report_ms = time.ticks_ms()


def maybe_report():
    global _last_report_ms
    now = time.ticks_ms()
    if time.ticks_diff(now, _last_report_ms) >= REPORT_MS:
        poll_encoders()  # counts are absolute; only read them when reporting
        # One write per report keeps the four lines together.
        sys.stdout.write("FL %d\nFR %d\nBL %d\nBR %d\n" % (
            _ENC_COUNTS[0], _ENC_COUNTS[1], _ENC_COUNTS[2], _ENC_COUNTS[3]))
        _last_report_ms = now

# -------------------- Command parsing --------------------

last_motor_ms = time.ticks_ms()
failsafe_on = False

def is_float(text):
    try:
        float(text)
        return True
    except ValueError:
        return False

def apply_wheel_line(line):
    # "fl,fr,bl,br" float line (-1..1 each) -> per-wheel PWM pulses.
    parts = line.split(',')
    if len(parts) != 4:
        return False
    if not all(is_float(p) for p in parts):
        return False

    fl, fr, bl, br = [float(p) for p in parts]

    # Reject values too far out (corrupted line or upstream bug), and NaN
    # or inf: the "<=" form is False for NaN, and int(nan * 400) would
    # otherwise raise and kill the loop.
    if not (abs(fl) <= 1.2 and abs(fr) <= 1.2 and
            abs(bl) <= 1.2 and abs(br) <= 1.2):
        return False

    set_pulse('fl', float_to_pulse(fl))
    set_pulse('fr', float_to_pulse(fr))
    set_pulse('bl', float_to_pulse(bl))
    set_pulse('br', float_to_pulse(br))
    return True

def apply_line(line):
    global on_ms, last_motor_ms, failsafe_on
    line = line.strip().lower()

    if len(line) == 2 and line.isdigit():
        on_ms = int(line) * 10
        return  # LED command, does not feed the motor failsafe

    if apply_wheel_line(line):
        last_motor_ms = time.ticks_ms()
        failsafe_on = False

# -------------------- Serial reading --------------------

MAX_CHARS_PER_PASS = 64  # a flood of input cannot starve the main loop

current_line = ''
newest_line = ''

poll = select.poll()
poll.register(sys.stdin, select.POLLIN)

def read_serial_newest_only():
    global current_line, newest_line
    for _ in range(MAX_CHARS_PER_PASS):
        if not poll.poll(0):
            break
        c = sys.stdin.read(1)
        if not c:
            break
        if c == '\n':
            newest_line = current_line
            current_line = ''
        elif c != '\r':
            current_line += c
            if len(current_line) > 80:
                current_line = ''  # oversized line: drop

    if newest_line:
        apply_line(newest_line)
        newest_line = ''

# -------------------- Main loop --------------------
#
# Non-blocking: LED on/off phases are timestamp-based (visible behavior
# unchanged: on for on_ms, off for 200 ms). Encoder decoding happens in PIO
# hardware and is unaffected by this loop.

led.on()
led_on_phase = True
phase_start_ms = time.ticks_ms()

# Armed only now, after all setup, so a setup error shows a traceback at the
# REPL instead of a reboot loop. Cannot be stopped once started.
wdt = WDT(timeout=WDT_TIMEOUT_MS)

try:
    while True:
        wdt.feed()
        now = time.ticks_ms()

        read_serial_newest_only()

        if time.ticks_diff(now, last_motor_ms) > FAILSAFE_MS:
            if not failsafe_on:
                neutral_all()
                failsafe_on = True

        maybe_report()

        if led_on_phase:
            if time.ticks_diff(now, phase_start_ms) >= on_ms:
                led.off()
                led_on_phase = False
                phase_start_ms = now
        else:
            if time.ticks_diff(now, phase_start_ms) >= 200:
                led.on()
                led_on_phase = True
                phase_start_ms = now

        time.sleep_ms(1)
except BaseException as e:
    # Any crash: stop the motors, show why, and restart cleanly.
    neutral_all()
    sys.print_exception(e)
    time.sleep_ms(100)
    machine.reset()
