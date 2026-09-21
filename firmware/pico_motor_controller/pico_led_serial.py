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
# = one full A-cycle). 4x decoding counts every transition of A and B,
# so 4 * 537.7 = ~2150.8 decoded counts per output-shaft revolution.
#
# Decode: interrupt-driven, one shared handler scans all 8 pins on every
# edge. At robot speeds (< ~2 kHz transitions per encoder) this costs
# ~20-30% CPU worst case. No-load motor speed (~11 kHz per encoder) would
# drop counts; upgrade path is PIO if that ever matters.

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

_ENC_A_PINS = []
_ENC_B_PINS = []
_ENC_DIRS = []
_ENC_COUNTS = []
_ENC_A_STATES = []
_ENC_B_STATES = []

for _name, _a_gp, _b_gp, _dir in _ENCODER_DEFS:
    _a_pin = Pin(_a_gp, Pin.IN, Pin.PULL_UP)
    _b_pin = Pin(_b_gp, Pin.IN, Pin.PULL_UP)
    _ENC_A_PINS.append(_a_pin)
    _ENC_B_PINS.append(_b_pin)
    _ENC_DIRS.append(_dir)
    _ENC_COUNTS.append(0)
    # Latch the actual A/B state BEFORE arming the IRQ so startup cannot
    # produce a false count from the encoder's initial state.
    _ENC_A_STATES.append(_a_pin.value())
    _ENC_B_STATES.append(_b_pin.value())


def _encoder_irq(pin):
    # 4x decode: +1 when (old_a ^ new_b), else -1, per changed channel.
    for i in range(4):
        a = _ENC_A_PINS[i].value()
        b = _ENC_B_PINS[i].value()
        if a != _ENC_A_STATES[i]:
            _ENC_COUNTS[i] += (1 if (_ENC_A_STATES[i] ^ b) else -1) * _ENC_DIRS[i]
        if b != _ENC_B_STATES[i]:
            _ENC_COUNTS[i] += (1 if (a ^ _ENC_B_STATES[i]) else -1) * _ENC_DIRS[i]
        _ENC_A_STATES[i] = a
        _ENC_B_STATES[i] = b


for _pin in _ENC_A_PINS + _ENC_B_PINS:
    _pin.irq(handler=_encoder_irq,
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

while True:
    read_serial_newest_only()

    now = time.ticks_ms()
    if time.ticks_diff(now, last_motor_ms) > FAILSAFE_MS:
        if not failsafe_on:
            neutral_all()
            failsafe_on = True

    # LED blink: on on_ms, off 200 ms (behavior unchanged). Reporting
    # happens inside both phases so the rate stays ~10 Hz even though
    # the phases use blocking waits. Encoder counting is IRQ-driven and
    # unaffected by anything in this loop.
    led.on()
    t0 = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), t0) < on_ms:
        maybe_report()
        time.sleep_ms(10)

    led.off()
    t0 = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), t0) < 200:
        maybe_report()
        time.sleep_ms(10)
