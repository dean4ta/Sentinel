# Send test commands to the Pico over USB serial.
# Usage: .venv/bin/python led_sender.py
#
#   NN          -> LED on-time = NN * 10 ms
#   x/y/r + 0-255 -> GP21/GP20/GP19 pulse = 1000 + value * 1000/255 us
#   q           -> quit
import serial

PORT = "/dev/cu.usbmodem1101"
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1)

print(f"Connected to {PORT}.")
print("LED: 2 digits (e.g. 15 = 150ms on).")
print("PWM: x/y/r + value 0-255 (0=1000us, 128=~1500us, 255=2000us).")
print("q to quit.")

while True:
    v = input("> ").strip().lower()
    if v in ("q", "quit"):
        break

    ok_led = len(v) == 2 and v.isdigit()
    ok_pwm = (
        len(v) >= 2 and v[0] in "xyr"
        and v[1:].isdigit() and int(v[1:]) <= 255
    )

    if ok_led or ok_pwm:
        ser.write((v + "\n").encode())
    else:
        print("Need 2 digits (LED) or x/y/r + 0-255 (PWM), e.g. 15 or r255")
