# sentinel_drive

Gamepad drive stack: `/joy` → `/cmd_vel` → Pico serial, plus slip detection and wheel odometry. See workspace [README](../../README.md) for build steps.

## Run

```bash
ros2 launch sentinel_drive gamepad.launch.py            # drive, no recording
ros2 launch sentinel_drive gamepad.launch.py record:=true   # + rosbag
```

Starts `joy_node` (PS4, 20 Hz autorepeat), `gamepad_twist`, `cmd_vel_serial`, `wheel_slip_detector`, `wheel_odometry`. Params load from `config/drive_params.yaml`. In recording mode, any button press (e.g. X) on the controller ends the run and closes the bag.

Robot-side tuning helpers live in `utils/` at the repo root: `ground_drive_tuning.sh` (record + plot), `odometry_measurements_record.sh` / `odometry_measurements_replay.sh` (offline odometry pipeline comparison).

## Fusion (wheel odom + D435i gyro)

```bash
source ~/dean/rs2_ws/install/setup.bash      # source-built realsense2_camera (RSUSB)
ros2 launch sentinel_drive fusion.launch.py  # camera, static TF, EKF
```

Starts realsense2_camera (gyro only, raw `/camera/realsense2_camera/gyro/sample`), the `base_link -> camera_link` static TF (camera mounted upside down: roll 180°), and robot_localization's `ekf_node` (config `ekf.yaml`) fusing wheel velocities + gyro yaw rate, publishing `/wheel_imu_odom_filtered` and `odom -> base_link` TF. Run alongside the drive stack. Verify feeds first:

```bash
ros2 topic hz /camera/realsense2_camera/gyro/sample
ros2 topic hz /wheel_odom
```

## SLAM mapping (RPLIDAR A1 + fused odom)

```bash
source ~/dean/rs2_ws/install/setup.bash      # source-built realsense2_camera (RSUSB)
ros2 launch sentinel_drive sentinel_bringup.launch.py mapping:=true
```

One launch brings up the drive stack, rosbridge (:9090 for Foxglove) and `mapping.launch.py`: RPLIDAR A1 driver (`/dev/ttyUSB0`, 115200 baud → `/scan`), `base_link -> laser` static TF (lidar 6 in forward, 4 in above base_link, X facing −X: yaw π), the fusion stack, and slam_toolbox online-async mapping (`slam_params_online_async.yaml`: map in `map` frame, base `base_link`, odom `/wheel_imu_odom_filtered`). Drive slowly; loop closure locks in when the robot revisits an area.

Drive only (no lidar, no realsense, no localization):

```bash
ros2 launch sentinel_drive sentinel_bringup.launch.py localize:=false
```

Save the map (map_saver does not create directories; the built-in
`/slam_toolbox/save_map` service spins only 2 s for a `/map` message and
misses the 5 s publish window, so use the standalone saver):

```bash
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/room
```

Writes `/home/sentinel1/maps/room.yaml` + `room.pgm` (via the built-in map saver).

## Localization (AMCL on a saved map)

```bash
source ~/dean/rs2_ws/install/setup.bash      # realsense driver for the EKF gyro
# drive stack + lidar + fusion + rosbridge + AMCL localization (default)
ros2 launch sentinel_drive sentinel_bringup.launch.py
# or AMCL alone, alongside an already-running bringup:
ros2 launch sentinel_drive localization.launch.py
```

The default bringup launches the same lidar + EKF feeds as mapping (`mapping.launch.py slam:=false`) so AMCL gets `/scan` and the `odom -> base_link` TF. It serves the map (`map_file`, default `/home/sentinel1/maps/robotics_lab.yaml`) via nav2 map_server and localizes with amcl (`amcl_nav2_params.yaml`: likelihood-field model, omni motion model for mecanum odom, fused wheel+gyro odom TF, 12 m laser range). amcl publishes `map -> odom` and `/amcl_pose`. Give it the start pose once: Foxglove 3D panel → right-click → Publish pose estimate (or RViz 2D Pose Estimate). Until then there is no `map -> odom` TF and no robot pose appears.

Foxglove checklist: 3D panel fixed frame `map`, add topic layers `/map`, `/scan`, `/particlecloud` (AMCL particles), and toggle the `odom`/`base_link`/`laser` frames on under Frames. Then publish the pose estimate.

## Navigation (Nav2 on the saved map)

```bash
source ~/dean/rs2_ws/install/setup.bash
ros2 launch sentinel_drive sentinel_bringup.launch.py nav:=true
```

Same lidar + EKF feeds as localization, plus the full Nav2 stack (`navigation.launch.py` includes nav2_bringup with `amcl_nav2_params.yaml`): map_server + amcl (omni motion model, fused odom), NavFn planner, DWB controller wrapped in a rotation shim with `vy` enabled for mecanum strafe, costmaps using the 0.508 x 0.406 m footprint, and the standard navigate-to-pose behavior tree. Max speeds 0.5 m/s / 1.0 rad/s (nav only — teleop ceilings untouched).

Use: publish the initial pose first (same as localization), then send a goal — Foxglove 3D panel right-click → Publish pose estimate → topic `/goal_pose` (or RViz 2D Nav Goal). Watch `/plan`, `/local_plan`, `/cmd_vel`, and the `global_costmap`/`local_costmap` layers in Foxglove.

Live view from Mac (no ROS needed): Foxglove Studio → Open connection → Rosbridge → `ws://ts_sentinel:9090` (rosbridge launched with `ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090`). 3D panel: fixed frame `map`, layers `/scan` + `/map`. The scan tracks the robot only while the fusion stack is up (odom → base_link TF).

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
