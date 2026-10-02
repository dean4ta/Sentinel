# sentinel_drive

Gamepad drive control: `/joy` → `/cmd_vel` → Pico serial. See workspace [README](../../README.md) for build steps.

## Run

```bash
ros2 launch sentinel_drive gamepad.launch.py
```

Starts `joy_node` (PS4 gamepad, 20Hz autorepeat), `gamepad_twist`, and `cmd_vel_serial`. Node params load from `config/drive_params.yaml`.

## Config

`config/drive_params.yaml` — all params below live here. Edit + relaunch; no CLI overrides needed.

## Calibration

TODO: PWM signals are not yet calibrated to actual velocity in SI units. `/cmd_vel` values are unitless drive commands in [-1,1]; need measurement of robot speed (m/s, rad/s) per command level and a mapping to true SI values.

## Measured wheel variance (reference)

Encoder magnitudes during forward drive, 2026-09-24 (cumulative counts, sign = side convention):

```
fl: -56667
fr:  35954
bl: -17136
br:  31146
```

Left wheels negative, right positive (encoder mounting faces opposite ways) — signs are consistent, not a bug. Magnitude spread is the problem: FL fastest, BL slowest (~0.3× FL). Left-side total exceeds right → robot veers right. Root cause for per-wheel direct control + gain correction (see wheel-gain plan).

## Nodes

`gamepad_twist`: subscribes `/joy`, maps axes (drive=axes[1], strafe=axes[0], rotate=axes[2]) with deadzone, publishes `geometry_msgs/Twist` on `/cmd_vel` in SI units (linear.x=drive m/s, linear.y=strafe m/s, angular.z=rotate rad/s), scaled by the same V_MAX ceilings `cmd_vel_serial` normalizes against.

`cmd_vel_serial`: bidirectional serial gateway. Writes mecanum IK wheel commands from `/cmd_vel` (per-wheel `fl,fr,bl,br` floats) to the Pico (channel remap + signs for Sentinel 1 wiring). Reads encoder count lines back and publishes `sentinel_msgs/EncoderCounts` (fl, fr, bl, br ints) on `/encoder`. Must be the only process holding the serial port.

## Params — gamepad_twist

| Param | Default | Meaning |
|---|---|---|
| `deadzone` | `0.05` | joystick deadzone, then rescaled |
| `invert_strafe` | `false` | flips strafe sign |
| `invert_rotate` | `false` | flips rotation direction |
| `rotate_limit` | `2.5` | clamps rotation, rad/s |
| `speed_scale` | `1.0` | scales all speeds, [0,1] |

## Params — cmd_vel_serial

| Param | Default | Meaning |
|---|---|---|
| `serial_port` | `/dev/ttyACM0` | Pico USB serial device |
| `baud` | `460800` | serial baud |
| `publish_rate` | `20.0` | Hz of serial writes |
| `ik_wheelbase_m` | `0.35` | half-track + half-wheelbase (m), mecanum IK geometry |
| `ik_roller_swap` | `false` | flips strafe/rotate IK terms for the opposite diagonal roller pairing |

Example override:

```bash
ros2 run sentinel_drive cmd_vel_serial --ros-args -p serial_port:=/dev/ttyACM1
```
