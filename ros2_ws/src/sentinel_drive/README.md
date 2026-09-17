# sentinel_drive

Gamepad → Pico serial drive control. See workspace [README](../../README.md) for build steps.

## Run

```bash
ros2 launch sentinel_drive gamepad.launch.py
```

Starts `joy_node` (PS4 gamepad, 20Hz autorepeat) + `gamepad_serial` (serial to Pico at `/dev/ttyACM0`).

## Params (all in `gamepad_serial`)

| Param | Default | Meaning |
|---|---|---|
| `serial_port` | `/dev/ttyACM0` | Pico USB serial device |
| `baud` | `460800` | serial baud |
| `deadzone` | `0.05` | joystick deadzone, then rescaled |
| `invert_strafe` | `false` | flips strafe sign |
| `rotate_limit` | `0.5` | clamps rotation speed |
| `publish_rate` | `20.0` | Hz of serial writes |

Example override:

```bash
ros2 run sentinel_drive gamepad_serial --ros-args -p serial_port:=/dev/ttyACM1 -p invert_strafe:=true
```

## Node

`gamepad_serial`: subscribes `/joy`, maps axes (drive=axes[1], strafe=axes[0], rotate=axes[2]), remaps channels for Sentinel 1 wiring, writes `drive,rotate,strafe\n` float triplets to Pico. Pico converts to PWM and enforces 200ms failsafe.
