# LED on-time + 4-wheel direct-drive PWM, controlled over USB serial.
# Also reads 4x quadrature wheel encoders (PIO, zero CPU involvement)
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
# Failsafe: no valid motor command for 200 ms -> all channels neutral.
# Watchdog: CPU wedged -> 500 ms WDT reset -> PWM stops; the GoBilda
# controller's own signal-loss failsafe stops the motors.
# Only the newest complete line is applied per cycle.
#
# PWM: 50 Hz (20 ms period), RC-style 1000-2000 us, 1500 = neutral.
from machine import Pin, PWM, WDT
import select, sys, time
import micropython
import rp2

# A stray 0x03 (Ctrl-C) byte on stdin must not kill the script while
# motors are spinning; interrupts stay off until reset.
micropython.kbd_intr(-1)

# Hardware watchdog: the main loop feeds it every pass. If the CPU
# wedges, the Pico resets and the PWM pins stop pulsing; the GoBilda
# controller's own signal-loss failsafe stops the motors. The WDT
# survives a soft reset, so use `mpremote reset` or power-cycle to stop
# the script.
WDT_TIMEOUT_MS = 500
wdt = WDT(timeout=WDT_TIMEOUT_MS)

led = Pin(25, Pin.OUT)
on_ms = 100  # default LED on-time: 10 x 10ms

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
# Decode: PIO, zero CPU involvement. One state machine per encoder runs
# the canonical RP2040 quadrature program (raspberrypi/pico-examples,
# pio/quadrature_encoder): a 10-cycle loop samples both pins, keeps the
# count in Y, and pushes it to the RX FIFO every loop. The main loop
# drains each FIFO and keeps the newest absolute count. Encoder noise
# can add bogus counts but can never starve the CPU (the 2026-09-29
# lockup: soft-IRQ decode wedged under brush-noise chatter at speed).
#
# IMPORTANT: the program jumps through a computed table at absolute
# addresses, so it must remain the ONLY asm_pio program in this file
# and the first one loaded (MicroPython places it at instruction
# offset 0).

@rp2.asm_pio(out_shiftdir=rp2.PIO.SHIFT_RIGHT)
def quadrature_encoder():
    # Computed-jump table, addresses 0-15: rows = new pin state
    # (A<<1)|B, entries = old state. Instruction order is part of the
    # encoding -- do not reorder.
    jmp("update")        # 00: read 00 -> no change
    jmp("decrement")     #     read 01
    jmp("increment")     #     read 10
    jmp("update")        #     read 11

    jmp("increment")     # 01: read 00
    jmp("update")        #     read 01
    jmp("update")        #     read 10
    jmp("decrement")     #     read 11

    jmp("decrement")     # 10: read 00
    jmp("update")        #     read 01
    jmp("update")        #     read 10
    jmp("increment")     #     read 11

    jmp("update")        # 11: read 00
    jmp("increment")     #     read 01
    label("decrement")
    jmp(y_dec, "update")  #    read 10: Y--, re-enter the loop

    wrap_target()
    label("update")
    mov(isr, y)          # 11: read 11 -> no change
    push(noblock)

    label("sample_pins")
    out(isr, 2)
    in_(pins, 2)
    mov(osr, isr)
    mov(pc, isr)

    label("increment")
    mov(y, invert(y))
    jmp(y_dec, "increment_cont")
    label("increment_cont")
    mov(y, invert(y))
    wrap()

# Set to -1 to flip one encoder's sign without touching wiring.
FL_ENCODER_DIRECTION = 1
FR_ENCODER_DIRECTION = 1
BL_ENCODER_DIRECTION = 1
BR_ENCODER_DIRECTION = 1

# name, A gpio, B gpio, direction. Pins per spec:
# FR A/B = GP6/7, BR A/B = GP8/9, FL A/B = GP10/11, BL A/B = GP12/13.
_ENCODER_DEFS = (
    ('FL', 10, 11, FL_ENCODER_DIRECTION),
    ('FR', 6, 7, FR_ENCODER_DIRECTION),
    ('BL', 12, 13, BL_ENCODER_DIRECTION),
    ('BR', 8, 9, BR_ENCODER_DIRECTION),
)

REPORT_MS = 20  # encoder report rate (50 Hz)

_ENC_COUNTS = [0, 0, 0, 0]
_ENC_DIRS = []
_SMS = []

for i, (_name, _a_gp, _b_gp, _dir) in enumerate(_ENCODER_DEFS):
    _a_pin = Pin(_a_gp, Pin.IN, Pin.PULL_UP)
    _b_pin = Pin(_b_gp, Pin.IN, Pin.PULL_UP)
    _ENC_DIRS.append(_dir)
    _sm = rp2.StateMachine(i, quadrature_encoder, in_base=_a_pin)
    _sm.active(1)
    _SMS.append(_sm)


def poll_encoders():
    # Drain each PIO FIFO, keep the newest absolute count. No IRQs:
    # a noise storm just means stale or bogus numbers, never a wedged
    # CPU. Y wraps at 2^31 steps (~4M revs), so the unsigned read is
    # sign-restored.
    for i, sm in enumerate(_SMS):
        if not sm.rx_fifo():
            continue
        val = sm.get()
        while sm.rx_fifo():
            val = sm.get()
        if val & 0x80000000:
            val -= 0x100000000
        _ENC_COUNTS[i] = val * _ENC_DIRS[i]


def print_report():
    for i, _name in enumerate(('FL', 'FR', 'BL', 'BR')):
        print(_name, _ENC_COUNTS[i])


_last_report_ms = time.ticks_ms()


def maybe_report():
    global _last_report_ms
    now = time.ticks_ms()
    if time.ticks_diff(now, _last_report_ms) >= REPORT_MS:
        print_report()
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

    # Reject values too far out: corrupted line or upstream bug.
    if max(abs(fl), abs(fr), abs(bl), abs(br)) > 1.2:
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

current_line = ''
newest_line = ''

def read_serial_newest_only():
    global current_line, newest_line
    while sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
        c = sys.stdin.read(1)
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
# unchanged: on for on_ms, off for 200 ms). Encoder counting happens in
# PIO hardware and is unaffected by this loop.

led.on()
led_on_phase = True
phase_start_ms = time.ticks_ms()

while True:
    wdt.feed()
    poll_encoders()
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
