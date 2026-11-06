#!/usr/bin/env python3
# RPLIDAR A1 motor brake. The A1 adapter runs the motor from the
# CP2102's DTR line, inverted: DTR LOW = spin, DTR HIGH = stop. A
# closed port drops DTR to LOW, so the lidar node's own shutdown can
# never hold the motor off - this daemon holds the port open and keeps
# DTR HIGH whenever no rplidar_node is running, re-asserting after the
# node dies (covers SIGKILL and crashes too).
#
# Runs as a systemd service (config/systemd/sentinel-lidar-brake.service)
# so it survives launch files and keeps the brake on across reboots.
import subprocess
import time

import serial

PORT = '/dev/sentinel_lidar'
POLL_S = 1.0


def node_running():
    return subprocess.run(
        ['pgrep', '-f', 'rplidar_node'],
        capture_output=True).returncode == 0


def main():
    while True:
        try:
            ser = serial.Serial(PORT, 115200)
        except serial.SerialException as exc:
            print(f'open failed ({exc}), retrying in 2 s')
            time.sleep(2.0)
            continue
        ser.dtr = True
        print(f'brake on: {PORT} held, DTR HIGH')
        while True:
            if not node_running():
                ser.dtr = True  # idempotent; keeps brake during idle
            time.sleep(POLL_S)


if __name__ == '__main__':
    main()
