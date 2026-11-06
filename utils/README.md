# utils — wheel-controller tuning tools

Loose scripts used during PID tuning. Not part of the ROS workspace on
purpose: after the per-wheel speed controllers are tuned, these get used
rarely. Data lives in repo-root `data/` (gitignored).

## Setup (on the Orin)

```bash
cd ~/dean/Sentinel/utils
python3 -m venv .venv
source .venv/bin/activate
pip install rosbags matplotlib
```

## Workflow

1. One launch does everything — serial gateway, rosbag recording, step
   test (bag starts 2 s before the test node, so step 1 lands at t=0):

   ```bash
   cd ~/dean/Sentinel/ros2_ws && source install/setup.bash
   cd ~/dean/Sentinel
   ros2 launch sentinel_drive_cpp step_test.launch.py \
       wheel:=FL steps:="0.2,5;0.5,10;1.0,5;0,1;-1.0,10"
   ```

   Ctrl-C after the test node prints "all steps done". Bag lands in
   `data/step_test_<timestamp>`.

2. Extract + plot:

   ```bash
   cd utils && source .venv/bin/activate
   python analyze_step_test.py ../data/step_test_2026-09-24_1430 \
       --wheel FL -o ../data/fl_step.png
   ```

   Output PNG: target rpm vs measured rpm over time, for the one wheel.

3. Adjust `fl_kp/ki/kd` (etc.) in `ros2_ws/src/sentinel_bringup/config/drive_params.yaml`, rebuild, rerun.

## Scripts

- `analyze_step_test.py` — reads a rosbag (via `rosbags`), extracts target +
  measured rpm for one wheel from `/wheel_speeds`, writes a matplotlib PNG.

## Odometry measurement workflow

Record raw inputs once (drive a route; X on the PS4 ends the run):

```bash
./utils/odometry_measurements_record.sh
```

Records `/wheel_speeds`, `/encoder`, `/camera/realsense2_camera/gyro/sample`
and `/tf_static` to `data/odom_input` on the Orin. Gyro-only by design:
the EKF fuses yaw rate, nothing else from the IMU.

Replay offline (4 passes, on the Orin):

```bash
./utils/odometry_measurements_replay.sh [input_bag] [rate]
```

| Pass | Pipeline | Output |
|---|---|---|
| `raw_odom` | wheel_odometry | `/wheel_odom` |
| `slip_odom` | slip detector + wheel_odometry | `/wheel_odom` |
| `raw_ekf` | wheel_odometry + ekf_node | `/wheel_imu_odom_filtered` |
| `slip_ekf` | slip detector + wheel_odometry + ekf_node | `/wheel_imu_odom_filtered` |

Pulls the four result bags back and plots them overlaid
(`data/replay/compare.png`).

Robot on blocks for all step tests.
