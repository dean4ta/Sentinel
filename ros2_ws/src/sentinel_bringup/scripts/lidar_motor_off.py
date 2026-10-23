#!/usr/bin/env python3
# Stop the RPLIDAR A1 motor. The CP2102 adapter drives the motor from
# its DTR line: DTR low = motor off. Runs from a launch file's
# OnShutdown handler so the motor stops even when the lidar node was
# killed hard (SIGKILL/crash skips its own setMotorSpeed(0) path).
import sys
import time

import serial

port = sys.argv[1] if len(sys.argv) > 1 else '/dev/sentinel_lidar'
for _ in range(20):  # up to 10 s: port may be held while the node dies
    try:
        ser = serial.Serial(port, 115200)
        ser.dtr = False
        ser.close()
        print(f'lidar motor off: {port}')
        break
    except serial.SerialException:
        time.sleep(0.5)
else:
    print(f'could not open {port}', file=sys.stderr)
