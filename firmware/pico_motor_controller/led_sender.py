# Send test commands to the Pico over USB serial.
# Usage: .venv/bin/python led_sender.py
#
#   NN          -> LED on-time = NN * 10 ms
#   xNN / yNN / rNN -> GP21/GP20/GP19 pulse = 1000 + NN*10 us
#   q           -> quit
import serial

PORT = "/dev/cu.usbmodem1101"
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1)

print(f"Connected to {PORT}.")
print("LED: 2 digits (e.g. 15 = 150ms on).")
print("PWM: x/y/r + 2 digits (00=1000us, 50=1500us, 99=1990us).")
print("q to quit.")

while True:
    v = input("> ").strip().lower()
    if v in ("q", "quit"):
        break

    ok_led = len(v) == 2 and v.isdigit()
    ok_pwm = len(v) == 3 and v[0] in "xyr" and v[1:].isdigit()

    if ok_led or ok_pwm:
        ser.write((v + "\n").encode())
    else:
        print("Need 2 digits (LED) or x/y/r + 2 digits (PWM), e.g. 15 or r99")
