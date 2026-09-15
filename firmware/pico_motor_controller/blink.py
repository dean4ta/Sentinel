# Blink the Pico onboard LED (GP25): 0.25s on, 0.25s off, repeat.
from machine import Pin
import time

led = Pin(25, Pin.OUT)

while True:
    led.value(1)
    time.sleep(0.25)
    led.value(0)
    time.sleep(0.25)
