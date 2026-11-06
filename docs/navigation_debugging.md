# Navigation Debugging

Status: paused 2026-11-06 (switching to arm operation). Resume here.

## Problems (from operator notes)

1. **Goal criteria too tight** — robot reaches the goal area, but the
   evaluation decides it hasn't arrived and runs spin/backup recoveries.
   The recoveries then degrade localization accuracy.
2. **Recovery commands too weak** — spin/backup sometimes produce
   commanded velocities too low to move the robot; it "wiggles" in place
   without enough power.

## Infrastructure built (in `sentinel_bringup`)

- `note_taker` — interactive terminal note entry. `n` starts a note
  (timestamp captured at keypress), `enter` publishes to
  `/programmers_notes`, `ctrl+x` cancels, `ctrl+c` quits.
- `programmer_notation` — launched by `sentinel_bringup.launch.py` when
  `take-notes:=true`. Subscribes to `/programmers_notes`, writes
  `~/programmer_notes_<timestamp>.txt` with ROS timestamps.
- `debug_nav.launch.py` — curated `ros2 bag record` for nav debugging.
  Bag lands in `~/bags/nav_debug_<timestamp>/`.
- `log_level` launch arg on `sentinel_bringup.launch.py` and
  `navigation.launch.py`, passed through to nav2_bringup. Default
  `info`. `log_level:=debug` floods the console and wedges ctrl+c
  (launch output pipe backs up) — if debug is needed, redirect:
  `... log_level:=debug > ~/nav_debug_console.log 2>&1`.

Note: Humble has no `set_logger_levels` service
(`rcl_interfaces/msg/LoggerLevel` doesn't exist) — per-node logger
config at runtime isn't possible on this distro. nav2_bringup's
`log_level` arg is the only lever.

## Run procedure

1. Bringup:
   `ros2 launch sentinel_bringup sentinel_bringup.launch.py nav:=true take-notes:=true`
2. Recorder: `ros2 launch sentinel_bringup debug_nav.launch.py`
3. Notes: `ros2 run sentinel_bringup note_taker`
4. Run the failing scenario; press `n` + note at every anomaly.
5. Stop bag + notes. Artifacts:
   - bag: `~/bags/nav_debug_<ts>/`
   - notes: `~/programmer_notes_<ts>.txt`
   - node logs: `~/.ros/log/` (one dir per launch; INFO by default)

## Bag topics

`/tf`, `/tf_static`, `/odom`, `/amcl_pose`, `/scan`, `/map`, `/plan`,
`/local_plan`, `/transformed_global_plan`, `/cmd_vel`, `/goal_pose`,
`/initialpose`, `/navigate_to_pose/_action/feedback`,
`/navigate_to_pose/_action/status`, `/behavior_tree_log`,
`/wheel_speeds`, `/wheel_consistency`, `/wheel_odom`, `/encoder`,
`/local_costmap/costmap_raw`, `/global_costmap/costmap_raw`,
`/received_global_plan`, `/smooth_path`,
`/velocity_smoother/trajectory`

## Nodes to watch

controller_server (goal checker + DWB), planner_server, bt_navigator,
behavior_server (spin/backup), velocity_smoother, amcl; ours:
cmd_vel_serial, wheel_odometry, wheel_slip_detector.

## First suspects (`config/amcl_nav2_params.yaml`)

- Problem 2: `min_speed_xy: 0.0` and `min_speed_theta: 0.0` (DWB) —
  controller can emit sub-friction velocities. Try `min_speed_xy` 0.15-0.2.
- Problem 1: `xy_goal_tolerance: 0.40`, `yaw_goal_tolerance: 0.50`
  (general_goal_checker) — post-recovery localization drift may keep the
  pose outside tolerance. Correlate `/amcl_pose` vs goal during recoveries.
- Also check velocity_smoother min/max params and the backup recovery's
  configured speed.

## Analysis approach

Correlate note timestamps → bag (`/behavior_tree_log` shows recovery
entry/exit, `/navigate_to_pose/_action/feedback` shows
distance_remaining when goal checker fails) → node logs. Measure
commanded `/cmd_vel` vs actual `/wheel_speeds` during wiggle episodes to
separate "commanded too low" from "drive can't deliver".
