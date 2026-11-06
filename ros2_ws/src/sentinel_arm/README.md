# sentinel_arm

RoArm-M2-S serial <-> ROS bridge. Jumping-off point for arm programming.

## Run

```
source ~/dean/rs2_ws/install/setup.bash   # if realsense env needed
ros2 launch sentinel_arm arm_bridge.launch.py
```

Requires the udev rule installed (`/dev/sentinel_arm` symlink, see
`sentinel_bringup/config/udev/99-sentinel-usb.rules`).

## Topics

| Topic | Type | Direction | Meaning |
|---|---|---|---|
| `arm/joint_states` | `sensor_msgs/JointState` | out, ~10 Hz | pose from `{"T":105}` |
| `arm/joint_goals` | `sensor_msgs/JointState` | in | per-joint move via `{"T":101}` |

Joint names (radians): `base`, `shoulder`, `elbow`, `clamp`.

## Quick test

```
ros2 topic echo arm/joint_states
ros2 topic pub arm/joint_goals sensor_msgs/msg/JointState \
  "{name: [base], position: [0.3]}"
```

## Notes

- Opening the serial port resets the ESP32 (DTR/RTS). The bridge drains
  the ~8 s boot/init sequence before commanding.
- Command set reference:
  https://www.waveshare.com/wiki/RoArm-M2-S_JSON_Command_Meaning
