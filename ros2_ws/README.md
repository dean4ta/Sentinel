# ros2_ws — Sentinel 1 ROS 2 Workspace

In-repo workspace for our own packages (prior team's workspace lives on the Orin at `~/ros2_ws`, separate).

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

- [sentinel_drive](src/sentinel_drive/README.md) — gamepad drive control
