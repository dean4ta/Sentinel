# LED on-time + 3ch mecanum mixer PWM, controlled over USB serial.
#
# Commands (one per line):
#   NN                     -> LED on-time = NN * 10 ms, off 200 ms, repeat
#   xNNN / yNNN / rNNN     -> GP21/GP20/GP19 pulse = 1000 + NNN * 1000/255 us
#                             (0-255, bench test format)
#   drive,rotate,strafe    -> float triplet (-1..1), production format from
#                             the Orin gamepad_serial node.
#                             drive -> GP21, rotate -> GP19, strafe -> GP20,
#                             pulse = 1500 + value * 500 us.
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

    led.on()
    time.sleep_ms(on_ms)
    led.off()
    time.sleep_ms(200)
