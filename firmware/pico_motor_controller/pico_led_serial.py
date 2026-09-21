# LED on-time + 3ch mecanum mixer PWM, controlled over USB serial.
# Also reads 4x quadrature wheel encoders (interrupt-driven) and reports
# counts over serial at ~10 Hz.
#
# Commands (one per line):
#   NN                     -> LED on-time = NN * 10 ms, off 200 ms, repeat
#   xNNN / yNNN / rNNN     -> GP21/GP20/GP19 pulse = 1000 + NNN * 1000/255 us
#                             (0-255, bench test format)
#   drive,rotate,strafe    -> float triplet (-1..1), production format from
#                             the Orin cmd_vel_serial node.
#                             drive -> GP21, rotate -> GP19, strafe -> GP20,
#                             pulse = 1500 + value * 500 us.
#
# Encoder report (printed ~10 Hz, one block of four lines):
#   FL <count>
#   FR <count>
#   BL <count>
#   BR <count>
#
# Failsafe: no valid motor command for 200 ms -> all channels neutral.
# Only the newest complete line is applied per cycle.
#
# PWM: 50 Hz (20 ms period), RC-style 1000-2000 us, 1500 = neutral.
from machine import Pin, PWM
import select, sys, time

led = Pin(25, Pin.OUT)
on_ms = 100  # default LED on-time: 10 x 10ms

# -------------------- Mecanum mixer PWM --------------------

PWM_FREQ = 50
PWM_PERIOD_US = 1000000 // PWM_FREQ  # 20000 us
NEUTRAL_US = 1500
VALUE_MAX = 255
FAILSAFE_MS = 200

CHANNEL_PINS = {
    "x": 21,
    "y": 20,
    "r": 19,
}

pwm = {}
for ch, gpio in CHANNEL_PINS.items():
    pwm[ch] = PWM(Pin(gpio))
    pwm[ch].freq(PWM_FREQ)

def set_pulse(ch, us):
    duty = int(65535 * us / PWM_PERIOD_US)
    pwm[ch].duty_u16(duty)

def value_to_pulse(v):
    # Map 0-255 linearly onto 1000-2000 us.
    return 1000 + (v * 1000) // VALUE_MAX

def float_to_pulse(v):
    # Map -1..1 onto 1000-2000 us, 0 = neutral.
    v = max(-1.0, min(1.0, v))
    return NEUTRAL_US + int(v * 500)

def neutral_all():
    for ch in CHANNEL_PINS:
        set_pulse(ch, NEUTRAL_US)

# Start all channels at neutral.
neutral_all()

# -------------------- Quadrature encoders --------------------
#
# goBILDA 5203 (19.2:1) encoders: 537.7 PPR at the output shaft (one PPR
# = one full A-cycle). 4x decoding counts every valid A/B transition,
# so 4 * 537.7 = ~2150.8 decoded counts per output-shaft revolution.
#
# Decode: interrupt-driven, 2-bit state table (state = (A<<1)|B). Each
# encoder has its own IRQ handler closure: an edge on FR pins decodes FR
# only, never the other three. Handlers are a few instructions; if counts
# ever drop at very high motor RPM the upgrade path is PIO.

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

REPORT_MS = 100  # encoder report rate (10 Hz)

# 4x decode table: delta for previous 2-bit state -> current 2-bit state.
# Rows = previous state, columns = current state (both = (A<<1)|B).
# Valid single-step transitions give +1/-1; impossible double-step
# transitions (00<->11, 01<->10) give 0.
_TRANSITION = (
    (0, -1, 1, 0),    # from 00
    (1, 0, 0, -1),    # from 01
    (-1, 0, 0, 1),    # from 10
    (0, 1, -1, 0),    # from 11
)

_ENC_A_PINS = []
_ENC_B_PINS = []
_ENC_DIRS = []
_ENC_COUNTS = []
_ENC_STATES = []  # 2-bit state (A<<1)|B, latched before IRQs arm

for _name, _a_gp, _b_gp, _dir in _ENCODER_DEFS:
    _a_pin = Pin(_a_gp, Pin.IN, Pin.PULL_UP)
    _b_pin = Pin(_b_gp, Pin.IN, Pin.PULL_UP)
    _ENC_A_PINS.append(_a_pin)
    _ENC_B_PINS.append(_b_pin)
    _ENC_DIRS.append(_dir)
    _ENC_COUNTS.append(0)
    # Latch the actual A/B state BEFORE arming the IRQ so startup cannot
    # produce a false count from the encoder's initial state.
    _ENC_STATES.append((_a_pin.value() << 1) | _b_pin.value())


def _decode(i):
    # Decode only encoder i: read its two pins, table-lookup the delta.
    cur = (_ENC_A_PINS[i].value() << 1) | _ENC_B_PINS[i].value()
    delta = _TRANSITION[_ENC_STATES[i]][cur]
    if delta:
        _ENC_COUNTS[i] += delta * _ENC_DIRS[i]
    _ENC_STATES[i] = cur


def _make_irq_handler(i):
    # One handler per encoder: an edge on either of its pins decodes
    # that encoder only. Stationary encoders are never touched.
    def handler(pin):
        _decode(i)
    return handler


for _i in range(4):
    _handler = _make_irq_handler(_i)
    _ENC_A_PINS[_i].irq(handler=_handler,
                        trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING)
    _ENC_B_PINS[_i].irq(handler=_handler,
                        trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING)


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

def apply_float_triplet(line):
    parts = line.split(',')
    if len(parts) != 3:
        return False
    if not all(is_float(p) for p in parts):
        return False

    drive, rotate, strafe = [float(p) for p in parts]

    # Reject values too far out: corrupted line or upstream bug.
    if max(abs(drive), abs(rotate), abs(strafe)) > 1.2:
        return False

    set_pulse('x', float_to_pulse(drive))
    set_pulse('r', float_to_pulse(rotate))
    set_pulse('y', float_to_pulse(strafe))
    return True

def apply_line(line):
    global on_ms, last_motor_ms, failsafe_on
    line = line.strip().lower()

    if len(line) == 2 and line.isdigit():
        on_ms = int(line) * 10
        return  # LED command, does not feed the motor failsafe

    motor_ok = False
    if line and line[0] in CHANNEL_PINS and line[1:].isdigit():
        v = int(line[1:])
        if v <= VALUE_MAX:
            set_pulse(line[0], value_to_pulse(v))
            motor_ok = True
    else:
        motor_ok = apply_float_triplet(line)

    if motor_ok:
        last_motor_ms = time.ticks_ms()
        failsafe_on = False

# -------------------- Serial reading --------------------

current_line = ''
newest_line = ''

poll = select.poll()
poll.register(sys.stdin, select.POLLIN)

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
# unchanged: on for on_ms, off for 200 ms). Encoder counting is
# IRQ-driven and unaffected by this loop.

led.on()
led_on_phase = True
phase_start_ms = time.ticks_ms()

while True:
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
