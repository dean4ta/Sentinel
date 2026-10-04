# sentinel_drive

Gamepad drive stack: `/joy` → `/cmd_vel` → Pico serial, plus slip detection and wheel odometry. See workspace [README](../../README.md) for build steps.

## Run

```bash
ros2 launch sentinel_drive gamepad.launch.py            # drive, no recording
ros2 launch sentinel_drive gamepad.launch.py record:=true   # + rosbag
```

Starts `joy_node` (PS4, 20 Hz autorepeat), `gamepad_twist`, `cmd_vel_serial`, `wheel_slip_detector`, `wheel_odometry`. Params load from `config/drive_params.yaml`. In recording mode, any button press (e.g. X) on the controller ends the run and closes the bag.

Robot-side tuning helpers live in `utils/` at the repo root: `ground_drive_tuning.sh` (record + plot), `slip_compensation_record.sh` / `slip_compensation_replay.sh` (offline slip-compensation comparison).

## Fusion (wheel odom + D435i IMU)

```bash
ros2 launch sentinel_drive fusion.launch.py    # camera, static TF, EKF
```

Starts realsense2_camera (gyro+accel only, united `/camera/imu`), the `base_link -> camera_link` static TF (camera mounted upside down: roll 180°), and robot_localization's `ekf_node` (config `ekf.yaml`) fusing wheel velocities + gyro yaw rate, publishing `/wheel_imu_odom_filtered` and `odom -> base_link` TF. Requires `ros-humble-robot-localization` and `ros-humble-realsense2-camera`. Run alongside the drive stack; verify the IMU topic matches `ekf.yaml` first:

```bash
ros2 topic echo --once /camera/imu | head -20
```

## Tests

```bash
python3 test/test_wheel_consistency.py   # slip detector math
python3 test/test_odometry_math.py       # odometry math
```

## Nodes

- `gamepad_twist` — `/joy` → `geometry_msgs/Twist` on `/cmd_vel` (SI: linear.x/y m/s, angular.z rad/s), deadzone + per-axis invert + rotate clamp + speed scale.
- `cmd_vel_serial` — `/cmd_vel` → mecanum IK → per-wheel PID+ff → `"fl,fr,bl,br\n"` over USB serial to the Pico. Reads encoder lines back, publishes `EncoderCounts` on `/encoder` and `WheelSpeeds` (target/measured/cmd rpm) on `/wheel_speeds`. Sole owner of the serial port. `/wheel_cmd` debug topic ("FL 30" = 30 rpm) overrides `/cmd_vel` while fresh.
- `wheel_slip_detector` — leave-one-out mecanum consistency on `/wheel_speeds` (FK from 3 wheels → predict 4th → residual). Asymmetric + sign-aware: only overspeeding wheels are suspicious; target-excess gate attributes slip and kills leave-one-out smear; tracking-error advantage scales the score. Publishes `WheelConsistency` (Header + per-wheel confidence 0-1 + residual m/s) on `/wheel_consistency`, filtered with asymmetric rates (fast decay, slow recovery), processed once per `/wheel_speeds` sample. Math: `wheel_consistency.py`.
- `wheel_odometry` — `/wheel_speeds` + `/wheel_consistency` → confidence-weighted ridge-regularised LS twist → integrated pose → `nav_msgs/Odometry` on `/wheel_odom`. Unobservable directions fall back to the previous estimate when wheels are distrusted. No TF (future fusion node owns odom→base). Math: `odometry_math.py`.

## Params

All in `config/drive_params.yaml`. Edit + relaunch.

### gamepad_twist

| Param | Default | Meaning |
|---|---|---|
| `deadzone` | `0.05` | joystick deadzone, then rescaled |
| `invert_strafe` | `false` | flips strafe sign |
| `invert_rotate` | `false` | flips rotation direction |
| `rotate_limit` | `6.0` | clamps rotation, rad/s |
| `speed_scale` | `0.5` | scales all speeds, [0,1] |
| `exit_on_x` | `false` | any button press ends the run (record mode sets true) |

### cmd_vel_serial

| Param | Default | Meaning |
|---|---|---|
| `serial_port` | `/dev/sentinel_pico` | Pico USB serial device |
| `baud` | `460800` | serial baud |
| `publish_rate` | `50.0` | Hz of serial writes |
| `ik_wheelbase_m` | `0.35` | half-track + half-wheelbase (m), mecanum IK geometry |
| `ik_roller_swap` | `false` | flips strafe/rotate IK terms for the opposite diagonal roller pairing |
| `pid_enabled` | `true` | PID block enable (independent of feed-forward) |
| `feed_forward_enabled` | `true` | ff block enable (independent of PID) |
| `{wheel}_kp/_ki/_kd` | per wheel | PID gains, error rpm → normalized PWM |
| `{wheel}_kS/_kV` | per wheel | ff: `u = kS*sign(v_ref) + kV*v_ref` |
| `{wheel}_output_limit` | per wheel | cap on commanded PWM |
| `{wheel}_ema_alpha` | per wheel | measured-rpm EMA blend |

### wheel_slip_detector

| Param | Default | Meaning |
|---|---|---|
| `wheel_circumference_m` | `0.44` | rpm → m/s conversion |
| `residual_threshold_mps` | `0.05` | residual (projected onto direction of travel) below this is ignored |
| `residual_scale_mps` | `0.15` | residual beyond threshold+this = full suspicion |
| `tracking_scale_mps` | `0.30` | tracking-error advantage over peer median that fully endorses suspicion |
| `peer_margin_mps` | `0.03` | wheel's target-excess must beat every peer's by this to count as suspicious |
| `alpha_suspicious` | `0.20` | confidence decay rate when suspicious (fast) |
| `alpha_recover` | `0.05` | confidence recovery rate (slow, hysteresis) |

### wheel_odometry

| Param | Default | Meaning |
|---|---|---|
| `wheel_circumference_m` | `0.44` | same rpm → m/s conversion |
| `lxy_m` | `0.362` | half-track + half-wheelbase; must match `ik_wheelbase_m` |
| `weight_floor` | `0.05` | distrusted wheel keeps this much weight |
| `ridge` | `0.05` | prior pull on unobservable directions |
| `wheel_noise_mps` | `0.02` | per-wheel speed noise, feeds covariance |
| `degraded_min_eig` | `0.5` | min eig of AᵀWA below this = degraded |
| `confidence_timeout_s` | `0.2` | stale confidences → equal weights |
| `max_dt_s` | `0.2` | longer gaps skip integration |
| `odom_frame` / `base_frame` | `odom` / `base_link` | frame names |

## Known gaps

- `V_MAX_LINEAR_MPS` / `V_MAX_ANGULAR_RADPS` / `WHEEL_MAX_RPM` in `cmd_vel_serial.py` are estimates (TODO: measure).
- `lxy_m` (0.362) vs `ik_wheelbase_m` (0.35) disagree ~3% on yaw; measure and unify.
- Mecanum roller slip makes strafe/rotate odometry drift even when healthy — validate against ground truth.
