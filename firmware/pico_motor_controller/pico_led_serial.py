# LED on-time + 3ch mecanum mixer PWM, controlled over USB serial.
#
# Commands (one per line):
#   NN        -> LED on-time = NN * 10 ms, off 200 ms, repeat (unchanged)
#   xNNN      -> GP21 (X) pulse width = 1000 + NNN * 1000/255 us
#   yNNN      -> GP20 (Y) pulse width = 1000 + NNN * 1000/255 us
#   rNNN      -> GP19 (rotation) pulse width = 1000 + NNN * 1000/255 us
#
# PWM: 50 Hz (20 ms period), RC-style 1000-2000 us, 1500 = neutral.
# NNN maps linearly over 0-255: 0->1000us, 128->1502us, 255->2000us.
from machine import Pin, PWM
import select, sys, time

led = Pin(25, Pin.OUT)
on_ms = 100  # default LED on-time: 10 x 10ms

# -------------------- Mecanum mixer PWM --------------------

PWM_FREQ = 50
PWM_PERIOD_US = 1000000 // PWM_FREQ  # 20000 us
NEUTRAL_US = 1500
VALUE_MAX = 255

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

# Start all channels at neutral.
for ch in CHANNEL_PINS:
    set_pulse(ch, NEUTRAL_US)

# -------------------- Serial --------------------

poll = select.poll()
poll.register(sys.stdin, select.POLLIN)

while True:
    # Non-blocking serial check between blink cycles.
    if poll.poll(0):
        line = sys.stdin.readline().strip().lower()

        if len(line) == 2 and line.isdigit():
            on_ms = int(line) * 10
        elif line and line[0] in CHANNEL_PINS and line[1:].isdigit():
            v = int(line[1:])
            if v <= VALUE_MAX:
                set_pulse(line[0], value_to_pulse(v))

    led.on()
    time.sleep_ms(on_ms)
    led.off()
    time.sleep_ms(200)
