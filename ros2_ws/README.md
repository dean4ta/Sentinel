# ros2_ws — Sentinel 1 ROS 2 Workspace

Our packages live in this repo. On the Orin, the checked-out copy is at `~/dean/Sentinel/ros2_ws` — build there (the Mac has no ROS 2).

## Build

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build
source install/setup.bash
```

Single package:

```bash
colcon build --packages-select sentinel_drive
```

`build/`, `install/`, `log/` are gitignored.

## Packages

- [sentinel_msgs](src/sentinel_msgs/README.md) — custom messages
- [sentinel_drive](src/sentinel_drive/README.md) — gamepad drive control, Pico serial gateway, slip detection, wheel odometry
