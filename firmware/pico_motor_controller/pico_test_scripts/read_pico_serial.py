#!/usr/bin/env python3
"""Print whatever the Pico writes on its USB serial port.

Default port matches the udev symlink in docs/pico_usage.md; override
with the first argument:
    .venv/bin/python pico_test_scripts/read_pico_serial.py /dev/ttyACM0

One process per port: stop cmd_vel_serial first. Ctrl-C to exit.
"""
import sys

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else '/dev/tty.usbmodem12301'
BAUD = 460800

ser = serial.Serial(PORT, BAUD, timeout=1)
try:
    while True:
        line = ser.readline()
        if line:
            sys.stdout.write(line.decode(errors='replace'))
            sys.stdout.flush()
except KeyboardInterrupt:
    pass
finally:
    ser.close()
