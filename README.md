# Sentinel

<!-- TODO: add a picture of the robot -->

Sentinel 1 is an autonomous retail security and assistance robot for the Loss Prevention Research Council. Inherited from prior teams as a gamepad-controlled mecanum platform, this semester's work turns it into an autonomous system that patrols store aisles, detects and reports hazards, and interacts with customers.

## Goals

1. **Mobile autonomy** — reliable localization (wheel encoders + IMU + LiDAR fusion) and navigation so the robot patrols a store unattended.
2. **Robot arm** — attach and program the arm with a pressure-sensing gripper to pick up small items.
3. **Customer and operator interface** — voice assistant and patrol mission control for store staff and customers.

## Repo layout

- `ros2_ws/` — ROS 2 Humble workspace (builds on the Orin). [README](ros2_ws/README.md)
  - `src/sentinel_msgs/` — custom messages (EncoderCounts, WheelSpeeds, WheelConsistency, StepEvent)
  - `src/sentinel_drive_cpp/` — C++ drive stack (gamepad → Pico gateway, slip detection, wheel odometry)
  - `src/sentinel_bringup/` — gamepad_twist, launch files, and shared config (drive, fusion, mapping, localization, navigation)
  - `src/sentinel_arm/` — RoArm-M2-S serial bridge (joint states / goals)
- `firmware/` — RP2040 Pico firmware (motors, encoders, IMU)
- `utils/` — scripts for tuning and analysis on the Mac (bag plotting, slip-compensation replay)
- `docs/` — robot research notes

## Quickstart (on the Orin)

```bash
cd ~/dean/Sentinel/ros2_ws
source /opt/ros/humble/setup.bash
colcon build && source install/setup.bash
cd ..
ros2 launch sentinel_drive_cpp gamepad.launch.py    # drive via PS4 controller
```

Details, params, and tests: [sentinel_bringup README](ros2_ws/src/sentinel_bringup/README.md).
