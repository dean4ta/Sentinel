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

`wheel_odometry`: subscribes `/wheel_speeds` + `/wheel_consistency`, solves a confidence-weighted ridge-regularised least squares for the body twist (falls back to the prior on unobservable directions when wheels are distrusted), integrates the pose, publishes `nav_msgs/Odometry` on `/wheel_odom`. No TF (future fusion node owns odom->base). Math in `odometry_math.py`, tests in `test/test_odometry_math.py`.

`wheel_slip_detector`: subscribes `/wheel_speeds`, runs a leave-one-out mecanum consistency check (FK from 3 wheels -> predict the 4th -> residual vs measured), publishes `sentinel_msgs/WheelConsistency` (std_msgs/Header + per-wheel confidence 0-1 and residual m/s) on `/wheel_consistency`. Asymmetric and sign-aware: only overspeeding wheels are suspicious (overspeed = larger magnitude along the wheel's direction of travel, so reverse slip also registers); a target-excess gate attributes the slip to the right wheel and kills leave-one-out smear; tracking-error advantage over the peer median scales the score. Filtered with asymmetric rates (fast decay, slow recovery). Processing runs in the subscription callback, so the filter advances once per `/wheel_speeds` sample. Started by `gamepad.launch.py`; no odometry node exists yet — a future one can weight its wheel-odom estimate with these confidences. Math lives in `wheel_consistency.py`, unit tests in `test/test_wheel_consistency.py` (`python3 test/test_wheel_consistency.py`).

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

## Params — wheel_slip_detector

| Param | Default | Meaning |
|---|---|---|
| `wheel_circumference_m` | `0.44` | rpm -> m/s conversion |
| `residual_threshold_mps` | `0.05` | residual (measured - predicted, projected onto direction of travel) below this is ignored |
| `residual_scale_mps` | `0.15` | residual beyond threshold+this = full suspicion |
| `tracking_scale_mps` | `0.30` | tracking-error advantage over peer median that fully endorses suspicion |
| `peer_margin_mps` | `0.03` | wheel's speed excess over its own target must beat every peer's excess by this to count as suspicious |
| `alpha_suspicious` | `0.20` | confidence decay rate when suspicious (fast) |
| `alpha_recover` | `0.02` | confidence recovery rate (slow, hysteresis) |

Example override:

```bash
ros2 run sentinel_drive cmd_vel_serial --ros-args -p serial_port:=/dev/ttyACM1
```
