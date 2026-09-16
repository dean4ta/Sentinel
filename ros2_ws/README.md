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

## Run

```bash
ros2 launch sentinel_drive gamepad.launch.py
```

Starts `joy_node` (PS4 gamepad, 20Hz autorepeat) + `gamepad_serial` (serial to Pico at `/dev/ttyACM0`).

Params (all in `gamepad_serial`):

| Param | Default | Meaning |
|---|---|---|
| `serial_port` | `/dev/ttyACM0` | Pico USB serial device |
| `baud` | `460800` | serial baud |
| `deadzone` | `0.05` | joystick deadzone, then rescaled |
| `invert_strafe` | `true` | flips strafe sign |
| `rotate_limit` | `0.5` | clamps rotation speed |
| `publish_rate` | `20.0` | Hz of serial writes |

Example override:

```bash
ros2 run sentinel_drive gamepad_serial --ros-args -p serial_port:=/dev/ttyACM1 -p invert_strafe:=false
```

## Node

`gamepad_serial`: subscribes `/joy`, maps axes (drive=axes[1], strafe=axes[0], rotate=axes[2]), writes `drive,rotate,strafe\n` float triplets to Pico. Pico converts to PWM and enforces 200ms failsafe.
