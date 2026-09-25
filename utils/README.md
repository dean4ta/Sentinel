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
   ros2 launch sentinel_drive step_test.launch.py \
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

3. Adjust `fl_kp/ki/kd` (etc.) in `ros2_ws/src/sentinel_drive/config/drive_params.yaml`, rebuild, rerun.

## Scripts

- `analyze_step_test.py` — reads a rosbag (via `rosbags`), extracts target +
  measured rpm for one wheel from `/wheel_speeds`, writes a matplotlib PNG.

Robot on blocks for all step tests.
