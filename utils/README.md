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

1. Start `cmd_vel_serial` (gamepad launch or standalone).
2. Record while a step test runs:

   ```bash
   cd ~/dean/Sentinel
   mkdir -p data
   ros2 bag record /wheel_speeds /encoder -o data/step_test_FL_$(date +%Y-%m-%d)
   ```

3. Extract + plot:

   ```bash
   cd utils && source .venv/bin/activate
   python analyze_step_test.py ../data/step_test_FL_2026-09-24 --wheel FL -o ../data/fl_step.png
   ```

   Output PNG: target rpm vs measured rpm over time, for the one wheel.

4. Adjust `fl_kp/ki/kd` (etc.) in `ros2_ws/src/sentinel_drive/config/drive_params.yaml`, rebuild, rerun.

## Scripts

- `analyze_step_test.py` — reads a rosbag (via `rosbags`), extracts target +
  measured rpm for one wheel from `/wheel_speeds`, writes a matplotlib PNG.

Robot on blocks for all step tests.
