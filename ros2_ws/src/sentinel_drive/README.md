# sentinel_drive

Gamepad drive control: `/joy` → `/cmd_vel` → Pico serial. See workspace [README](../../README.md) for build steps.

## Run

```bash
ros2 launch sentinel_drive gamepad.launch.py
```

Starts `joy_node` (PS4 gamepad, 20Hz autorepeat), `gamepad_twist`, and `cmd_vel_serial`. Node params load from `config/drive_params.yaml`.

## Config

`config/drive_params.yaml` — all params below live here. Edit + relaunch; no CLI overrides needed.

## Nodes

`gamepad_twist`: subscribes `/joy`, maps axes (drive=axes[1], strafe=axes[0], rotate=axes[2]) with deadzone, publishes `geometry_msgs/Twist` on `/cmd_vel` (linear.x=drive, linear.y=strafe, angular.z=rotate).

`cmd_vel_serial`: subscribes `/cmd_vel`, remaps channels + signs for Sentinel 1 wiring, writes `drive,rotate,strafe\n` float triplets to the Pico at a fixed rate. Pico converts to PWM and enforces 200ms failsafe.

## Params — gamepad_twist

| Param | Default | Meaning |
|---|---|---|
| `deadzone` | `0.05` | joystick deadzone, then rescaled |
| `invert_strafe` | `false` | flips strafe sign |
| `rotate_limit` | `0.5` | clamps rotation speed |

## Params — cmd_vel_serial

| Param | Default | Meaning |
|---|---|---|
| `serial_port` | `/dev/ttyACM0` | Pico USB serial device |
| `baud` | `460800` | serial baud |
| `publish_rate` | `20.0` | Hz of serial writes |

Example override:

```bash
ros2 run sentinel_drive cmd_vel_serial --ros-args -p serial_port:=/dev/ttyACM1
```
