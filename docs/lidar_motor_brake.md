# Lidar Motor Brake (RPLIDAR A1)

The A1's motor runs from the CP2102 adapter's DTR line, inverted:
DTR LOW = spin, DTR HIGH = stop. A closed serial port drops DTR to
LOW, so the lidar node's own shutdown can never hold the motor off —
and SIGKILL/crash skips its shutdown entirely.

Fix: a small daemon holds `/dev/sentinel_lidar` open and keeps DTR
HIGH whenever no `rplidar_node` is running. While the node scans, the
daemon leaves the line alone; once the node is gone (polled 1/s), it
re-asserts DTR HIGH and the motor stops.

Files:

- `ros2_ws/src/sentinel_bringup/scripts/lidar_motor_brake.py` — daemon
- `ros2_ws/src/sentinel_bringup/config/systemd/sentinel-lidar-brake.service`

## Install (on the Orin, after a git pull)

```bash
sudo cp ~/dean/Sentinel/ros2_ws/src/sentinel_bringup/config/systemd/sentinel-lidar-brake.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now sentinel-lidar-brake
systemctl status sentinel-lidar-brake
```

Service is bound to `dev-sentinel_lidar.device`: stops when the lidar
is unplugged, starts again on replug. Requires the udev rules
installed (see `udev_rules.md`).

## Verify

Motor must be stopped while idle. During any lidar launch it spins;
within ~1 s of launch exit it stops again:

```bash
systemctl status sentinel-lidar-brake   # active
journalctl -u sentinel-lidar-brake      # "brake on" lines
```
