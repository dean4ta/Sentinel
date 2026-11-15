# sentinel_debug

Debug/verification tools for the Sentinel 1 nav stack. First tool: the
**long-goal stress test**.

## What it does

`goal_stress_test` publishes ONE `/goal_pose` exactly the way Foxglove's
"click to goal" does, while bagging and monitoring the evidence topics, so
the TF-buffer goal-death diagnosis can be confirmed at runtime:

* Publishes one `PoseStamped` on `/goal_pose`: `header.frame_id =
  base_link`, position `x = +distance` (default 12 m, the robot's forward
  +X), identity quaternion, and a **whole-second stamp** (sec only,
  nanosec = 0 — Foxglove's click timestamp).
* Runs `ros2 bag record` as a subprocess (rosbag2 via CLI only, no
  rosbag2 Python dependency) on: `/goal_pose /tf /tf_static /plan
  /cmd_vel /amcl_pose /navigate_to_pose/_action/status /rosout`.
* Subscribes `/rosout` and writes every line to
  `<bag_dir>/rosout_filtered.txt` (prefixed with stamp + node name), and
  prints live any line containing `Extrapolation`,
  `transformPoseInTargetFrame` or `Aborting` (case-insensitive).
* Subscribes `/navigate_to_pose/_action/status`, logs state transitions
  with timestamps, and prints a heartbeat every 5 s (state + distance
  driven since start) so the run is visibly alive.
* After `duration` seconds: SIGINTs the bag, waits for a clean exit, and
  prints a summary (goal frame + stamp mode, extrapolation seen yes/no +
  first occurrence wall time, final action status, artifact paths).

## Root-cause hypothesis it tests

Foxglove publishes `/goal_pose` in `base_link` with a whole-second stamp.
Nav2's planner re-transforms that original stamped goal on every ~1 s
replan, and after ~10 s the stamp falls out of the TF buffer (nav2
default 10 s lookup window) → `transformPoseInTargetFrame` raises
"Extrapolation Error" → planner aborts → recoveries run → "Goal failed".

**Expected result (default run):** extrapolation error appears ~10 s
after the goal publish, the action aborts, recoveries run, and the
rosout_filtered.txt lines pin the timestamp. **A/B run**
(`frame:=map truncate_stamp:=false`): the goal survives past 10 s and the
robot drives the 12 m — no extrapolation lines.

## Run (on the robot, alongside the running bringup)

```bash
cd ~/Sentinel/ros2_ws
colcon build --packages-select sentinel_debug
source install/setup.bash

# Foxglove mimic (the bug reproduction):
ros2 launch sentinel_debug goal_stress_test.launch.py

# Healthy A/B control:
ros2 launch sentinel_debug goal_stress_test.launch.py \
    frame:=map truncate_stamp:=false
```

Launch args (underscores, not dashes):

| arg | default | meaning |
|-----|---------|---------|
| `frame` | `base_link` | goal header frame (map = healthy) |
| `truncate_stamp` | `true` | whole-second stamp like Foxglove (`false` = fresh stamp) |
| `duration` | `60.0` | run length in seconds |
| `distance` | `12.0` | goal x offset in meters |

Artifacts land in `~/bags/goal_stress_<timestamp>/`:
`rosout_filtered.txt` (grep it for `Extrapolation` / `Aborting`) plus the
bag (metadata.yaml + storage). The node also runs standalone:
`ros2 run sentinel_debug goal_stress_test --ros-args -p frame:=map -p
truncate_stamp:=false`.

**Safety:** the healthy A/B run drives the robot `distance` meters
forward. Clear the space, stand behind the robot, and be ready on the
E-stop.

## Mac selftest

```bash
cd ros2_ws/src/sentinel_debug
python3 test/test_goal_stress_test.py   # expect: ALL PASS
```

No ROS needed: the test AST-extracts everything above the `import rclpy`
seam in `goal_stress_test.py` (stamp truncation, `is_interesting`
predicate, log/summary formatting, status names, bag command), exercises
it with plain asserts, and grammar/bytecode-checks the module and the
launch file.
