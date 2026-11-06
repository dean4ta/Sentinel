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
colcon build --packages-select sentinel_bringup
```

`build/`, `install/`, `log/` are gitignored.

## Packages

- [sentinel_msgs](src/sentinel_msgs/README.md) — custom messages
- [sentinel_drive_cpp](src/sentinel_drive_cpp/README.md) — C++ drive nodes (cmd_vel_serial, wheel_slip_detector, wheel_odometry)
- [sentinel_bringup](src/sentinel_bringup/README.md) — gamepad_twist, launch files, shared config
