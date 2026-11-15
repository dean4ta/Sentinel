# sentinel_debug

Debug/verification tools for the Sentinel 1 nav stack.

1. **long-goal stress test** (`goal_stress_test`, T011)
2. **step recorder** (`step_recorder`, T012) — CSV capture of the
   controller-level step tests for the Mac-side analysis.

## goal_stress_test

Publishes ONE `/goal_pose` while bagging and
monitoring the evidence topics. The DEFAULT run is healthy: the goal is
12 m ahead of the robot's **current** amcl pose (map frame, current
heading, fresh stamp) — the robot really drives 12 m forward. The
Foxglove mimic is an explicit opt-in:

* Goal creation: the robot pose comes from TF (`map <- base_link`,
  latest time) — AMCL stops publishing `/amcl_pose` when the robot is
  stationary (update gates), so TF is the live source. If TF is
  unavailable, the latest `/amcl_pose` is used regardless of age (warned
  when > 5 s old); publishing is refused only when neither source exists
  after 10 s. `goal_from_pose(pose, distance, frame)` then computes the
  goal: map frame → `x + d·cos(yaw)`, `y + d·sin(yaw)`, orientation from
  yaw; base_link → x = distance, identity orientation.
* Runs `ros2 bag record` as a subprocess (rosbag2 via CLI only, no
  rosbag2 Python dependency) on: `/goal_pose /tf /tf_static /plan
  /cmd_vel /amcl_pose /navigate_to_pose/_action/status /rosout`.
* Subscribes `/rosout` and writes every line to
  `<bag_dir>/rosout_filtered.txt` (prefixed with stamp + node name), and
  prints live any line containing `Extrapolation`,
  `transformPoseInTargetFrame` or `Aborting` (case-insensitive).
* Subscribes `/navigate_to_pose/_action/status`, logs state transitions
  with timestamps, and prints a heartbeat every 5 s (state + distance
  driven since the goal publish).
* After `duration` seconds: SIGINTs the bag, waits for a clean exit, and
  prints a summary (goal frame + stamp mode, published yes/no,
  extrapolation seen yes/no + first occurrence wall time, final action
  status, artifact paths).

## Root-cause findings it reproduces

Two distinct failure mechanisms against nav2:

1. **Immediate recovery** (the bug observed on the first on-robot run):
   Foxglove's click stamp is a whole-second truncation (nanosec = 0), up
   to ~1 s in the past. `planner_server`'s `transform_tolerance` is
   0.2 s, so the first plan attempt re-transforming that stamp raises an
   "Extrapolation Error" instantly → immediate recovery.
2. **TF-buffer death**: a `base_link` goal with a FRESH (but frozen)
   stamp is re-transformed on every ~1 s replan; after ~10 s the stamp
   falls out of the TF buffer (nav2 default 10 s) → "Extrapolation
   Error" → planner abort → recoveries → "Goal failed".

## Run (on the robot, alongside the running bringup)

```bash
cd ~/Sentinel/ros2_ws
colcon build --packages-select sentinel_debug
source install/setup.bash

# Healthy default: goal 12 m ahead of the current pose, map frame,
# fresh stamp. The robot drives 12 m forward.
ros2 launch sentinel_debug goal_stress_test.launch.py

# Foxglove mimic (bug reproduction): expect FAST failure — immediate
# recovery, not the 10 s death.
ros2 launch sentinel_debug goal_stress_test.launch.py \
    frame:=base_link truncate_stamp:=true

# Isolated ~10 s TF-buffer death: base_link goal with a fresh stamp.
ros2 launch sentinel_debug goal_stress_test.launch.py \
    frame:=base_link truncate_stamp:=false
```

Launch args (underscores, not dashes):

| arg | default | meaning |
|-----|---------|---------|
| `frame` | `map` | goal frame (`base_link` = Foxglove mimic) |
| `truncate_stamp` | `false` | whole-second stamp like Foxglove (`true` = mimic) |
| `duration` | `60.0` | run length in seconds |
| `distance` | `12.0` | goal distance ahead, in meters |

Artifacts land in `~/bags/goal_stress_<timestamp>_<pid>/` (unique per
run; `-2`/`-3` suffixes if a collision somehow occurs):
`rosout_filtered.txt` (grep it for `Extrapolation` / `Aborting`) plus the
bag (metadata.yaml + storage). The node also runs standalone:
`ros2 run sentinel_debug goal_stress_test --ros-args -p frame:=base_link
-p truncate_stamp:=true`.

**Safety:** the healthy default run drives the robot `distance` meters
forward from its current pose. Clear the space, stand behind the robot,
and be ready on the E-stop.

## step_recorder (T012)

Records the step-test topics as CSVs for the Mac-side analysis
(`utils/controller_level_analysis.py` — the Mac has no ROS/rosbag2, so
the bag is not enough). One CSV per topic in `data/controller_level_<level>/`:

| CSV | topic | columns |
|-----|-------|---------|
| `wheel_speeds.csv` | `/wheel_speeds` | `t` + all 16 `WheelSpeeds` fields (target / measured filtered / measured raw / cmd rpm per wheel) |
| `wheel_odom.csv` | `/wheel_odom` | `t`, `x`, `y`, `yaw`, `vx`, `wz` |
| `wheel_imu_odom_filtered.csv` | `/wheel_imu_odom_filtered` | same odom schema |
| `step_events.csv` | `/step_events` | `t`, `index`, `target_mps`, `sine` |

`t` is the msg header stamp (odometry) or the receive time
(`WheelSpeeds`/`StepEvent` have no header). Flushed every 1 s and closed
on shutdown, so a Ctrl-C'd run keeps its data. Normally launched by
`step_test.launch.py` (`controller_level:=...`); standalone:
`ros2 run sentinel_debug step_recorder --ros-args -p level:=ffpid
-p out_dir:=data/controller_level_ffpid`.

## Mac selftest

```bash
cd ros2_ws/src/sentinel_debug
python3 test/test_goal_stress_test.py   # expect: ALL PASS
python3 test/test_step_recorder.py      # expect: ALL PASS
```

No ROS needed: the tests AST-extract everything above the `import rclpy`
seam in `goal_stress_test.py` / `step_recorder.py` (and above the
`import numpy` seam in `utils/controller_level_analysis.py`) — stamp
truncation, `is_interesting`, `goal_from_pose` + quaternion helpers,
`choose_pose` pose-source decision, bag-dir uniqueness, defaults,
log/summary formatting, status names, bag command (goal stress test);
CSV row formatting + field order, yaw-from-quaternion, odom-fallback
selection, step-event parsing, level-dir naming, a recorder-write ->
analysis-parse round trip, and the `controller_level` launch-file
defaults (step recorder) — with plain asserts, plus grammar/
bytecode-checks of the modules and launch files.
