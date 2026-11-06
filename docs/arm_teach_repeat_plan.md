# Arm Teach-and-Repeat — Architecture and Implementation Plan

Status: **proposal, nothing implemented**. For review before any code changes.
Branch: `feat/arm_testing`. Date: 2026-11-06.

---

## 1. Current Architecture Assessment

### What exists

| Piece | File | State |
|---|---|---|
| Hardware bridge | `sentinel_arm/sentinel_arm/arm_bridge.py` | One node, owns `/dev/sentinel_arm` serial port exclusively |
| Pose feedback | topic `arm/joint_states` (`sensor_msgs/JointState`) | ~10 Hz, from `{"T":105}` polls → `T:1051` responses |
| Move command | topic `arm/joint_goals` (`sensor_msgs/JointState`) | Fire-and-forget, per-joint `{"T":101,"joint":N,"rad":r,"spd":s,"acc":a}` |
| Launch | `arm_bridge.launch.py`; bringup also starts the arm node | Bridge params: port, baud, spd, acc, read_rate, home |
| Gripper sensors | `sentinel_msgs/msg/GripperSensors.msg` on `/gripper` | 10 Hz raw ADC from the motor-controller Pico |

Joints (radians): `base`=1, `shoulder`=2, `elbow`=3, `clamp`=4.

### What the arm hardware can do (RoArm-M2-S JSON commands)

| Command | Meaning | Used today |
|---|---|---|
| `T:101` | Move one joint to rad (spd/acc) | Yes, via `arm/joint_goals` |
| `T:105` / `T:1051` | Pose read / response (b,s,e,t + x,y,z) | Yes |
| `T:1041` | XYZT direct move (arm's own IK), used for home | Yes (home only) |
| `T:210` | Torque lock `cmd:0` (off = free manual motion), `cmd:1` (on) | **No — not exposed** |
| `T:220–224` | Onboard mission system: create mission, append steps (incl. `T:223` = record current position as a step), save to ESP32 flash | **No** |

### Gaps between today and the first milestone

1. **No completion feedback.** `arm/joint_goals` is fire-and-forget. Playback needs "joints reached target" detection. Fixable entirely host-side: bridge already publishes `arm/joint_states` at 10 Hz; a consumer can compare against the goal and wait.
2. **No torque release.** True manual teach (hand-guide the arm) needs `T:210 cmd:0`. One small bridge addition (a service).
3. **No waypoint persistence.** Nothing records or stores poses; nothing replays a sequence.
4. **No service/action interfaces anywhere in the repo.** Existing convention is topics only. This system introduces the first services.

---

## 2. Proposed Architecture

### Design decisions (and why)

1. **Joint space is authoritative; Cartesian is out of scope for v1.**
   - Playback replays exact recorded joint angles via `T:101` — deterministic, no IK ambiguity, no dependence on the arm's internal IK version.
   - The arm's Cartesian move (`T:1041`) depends on its firmware IK and can differ across firmware versions; also irrelevant for repeat of a taught pose.
   - Recorded Cartesian (`x,y,z` from `T:1051`) is deliberately *not* stored in v1 — see Risks #6.
2. **Waypoint files live on the host, not in ESP32 flash.** The arm's onboard `T:220–224` mission system is opaque (no per-step progress), lost on firmware flash, and not git-trackable. Host-side YAML is debuggable, portable, and satisfies the explicit save/load requirement. (Export to onboard missions listed as a later stretch goal.)
3. **One new node, two new files.** The proposed `teach_node` / `waypoint_recorder` / `trajectory_loader` / `trajectory_player` map to:
   - `waypoint_file.py` — pure-Python YAML I/O, **no rclpy import** (recorder + loader). Testable on the Mac without a robot.
   - `arm_teach.py` — one ROS node (teach + playback state machine). No serial code; it talks to the hardware only through the bridge's existing topics, plus one new bridge service for torque.
4. **Playback is step-based in v1.** Move → wait for convergence (joint states within tolerance) → optional hold → next. Interpolation (joint-space linear, host-side ticker) is a later phase with a per-waypoint `mode` field reserved.

### Component diagram

```
┌──────────────────────────────────────────────────────────────────┐
│ sentinel_arm package (ros2_ws)                                   │
│                                                                  │
│  ┌──────────────┐  serial  ┌──────────────────────────────┐      │
│  │ RoArm-M2-S   │◄────────►│ arm_bridge (existing,        │      │
│  │  (ESP32)     │          │   +1 new service)            │      │
│  └──────────────┘          │   arm/joint_states   (out)   │      │
│                            │   arm/joint_goals   (in)     │      │
│                            │   arm/torque        (new srv)│      │
│                            └───────────┬──────────────────┘      │
│                                        │                         │
│  ┌─────────────────────────────────────▼───────────────────┐     │
│  │ arm_teach (new node, no serial access)                  │     │
│  │   services: arm/record_waypoint, arm/waypoint_cmd,      │     │
│  │             arm/playback                                │     │
│  │   topics:   arm/teach_status (out)                      │     │
│  │   waypoint_file.py: YAML in ~/arm_waypoints/            │     │
│  └─────────────────────────────────────────────────────────┘     │
└──────────────────────────────────────────────────────────────────┘
```

### Interfaces (new)

All new service definitions go in `sentinel_msgs` (the repo's existing interface package, ament_cmake).

| Interface | Type | Definition | Purpose |
|---|---|---|---|
| `arm/torque` | service | `SetTorque.srv`: `bool enable` → `bool success` | Bridge sends `{"T":210,"cmd":1 or 0}`. `enable=false` releases motors for hand-guiding |
| `arm/record_waypoint` | service | `RecordWaypoint.srv`: `string name` → `bool success, string message` | Snapshots the latest `arm/joint_states` into the in-memory waypoint list |
| `arm/waypoint_cmd` | service | `WaypointCmd.srv`: `string cmd, string arg` → `bool success, string message` | Subcommands: `list`, `remove <name>`, `rename <old> <new>`, `reorder <name> <index>`, `clear`, `save <filename>`, `load <filename>`. One service with documented subcommands instead of seven typed services — deliberately; upgrade path noted in Risks #7 |
| `arm/playback` | service | `Playback.srv`: `string cmd, string filename` → `bool success, string message` | `start <filename>`, `pause`, `resume`, `stop` |
| `arm/teach_status` | topic | `std_msgs/String` | State machine output: `idle`, `playing 2/5: grasp`, `paused`, `done`, `error: <reason>` |

### Playback state machine (v1)

```
idle ──start(file)──► load YAML → validate joint_names
                        │
                        ▼
                    for each waypoint:
                      publish arm/joint_goals (all 4 joints)
                      wait: max|joint_state − goal| < tol  (default 0.02 rad)
                            for hold_samples consecutive polls (default 2)
                            else timeout (default 10 s) → error: abort
                      if delay_after: sleep
                        │
        ┌───────────────┼────────────────┐
        │ pause         │ stop           │ done
        ▼               ▼                ▼
      hold current     hold current    idle, "done"
      joints, stay     joints,         (keep last pose)
      at index         idle
```

- `pause` / `stop` / `resume` are services handled by the state machine; no ROS action needed for v1 (playback runs in the node's timer/spin thread, status broadcast on `arm/teach_status`).
- `pause` = command the *current* joints once (freeze), remember index. `resume` re-sends the goal of the paused index and continues.
- `stop` = freeze at current joints and return to idle; state cleared.
- Timeout → freeze at current joints, status `error: waypoint 'grasp' not reached in 10.0 s`.

### Files

| File | Change | Content |
|---|---|---|
| `sentinel_msgs/srv/SetTorque.srv` | new | bridge torque service |
| `sentinel_msgs/srv/RecordWaypoint.srv` | new | record service |
| `sentinel_msgs/srv/WaypointCmd.srv` | new | manage service |
| `sentinel_msgs/srv/Playback.srv` | new | playback service |
| `sentinel_msgs/CMakeLists.txt` | edit | add 4 srv to `rosidl_generate_interfaces` |
| `sentinel_arm/sentinel_arm/waypoint_file.py` | new | YAML load/save/edit, no rclpy |
| `sentinel_arm/sentinel_arm/arm_teach.py` | new | teach + playback node |
| `sentinel_arm/sentinel_arm/arm_bridge.py` | edit | +`arm/torque` service (~10 lines) |
| `sentinel_arm/launch/arm_teach.launch.py` | new | launches bridge + teach together |
| `sentinel_arm/setup.py` | edit | new entry point + launch data file |
| `sentinel_arm/package.xml` | edit | +depend `sentinel_msgs` |
| `sentinel_arm/test/test_waypoint_file.py` | new | YAML roundtrip self-check (runs on Mac, no robot) |

Bringup launch is *not* touched in v1 — arm work starts from `arm_teach.launch.py`. Adding `arm_teach` to `sentinel_bringup` is Phase 3.

---

## 3. Data Format

YAML, one file per trajectory, stored in `~/arm_waypoints/` (param `waypoint_dir`). Joint positions in radians, order fixed: `base, shoulder, elbow, clamp`.

```yaml
# ~/arm_waypoints/pick_place_demo.yaml
name: pick_place_demo
created: 2026-11-06T14:32:00
joint_names: [base, shoulder, elbow, clamp]   # validation header

waypoints:
  - name: home
    position: [0.00, 0.50, -0.30, 0.00]
    delay_after: 0.5                          # seconds, optional (default 0)

  - name: approach
    position: [0.35, 0.70, -0.60, 0.00]

  - name: grasp                          # clamp closes; delay lets it settle
    position: [0.35, 0.70, -0.60, 1.10]
    delay_after: 1.0

  - name: lift
    position: [0.00, 1.10, -0.90, 1.10]
```

Rules:

- `joint_names` must equal `[base, shoulder, elbow, clamp]` and every `position` must have 4 values — load-time validation, errors reject the file.
- `delay_after` is optional, default 0.
- Duplicate waypoint names rejected at `record`/`rename` time.
- Files are plain text; they can be committed to git like any other config.

**Deliberately absent from v1** (add only when used): per-waypoint `spd`/`acc` overrides (requires a bridge change — `JointState` goals carry no speed), Cartesian `xyz` reference, timestamps per waypoint, interpolation `mode` field.

---

## 4. Execution Plan

### Phase 0 — Interfaces and file I/O (Mac, no robot)

**Goal:** everything compiles, YAML roundtrip proven.

1. Add 4 `.srv` files to `sentinel_msgs` + CMakeLists entry. Mac-side `colcon build` must pass.
2. Write `waypoint_file.py` (pure Python + PyYAML): `load_file`, `save_file`, `add/remove/rename/reorder/list/clear` on an in-memory list.
3. Write `test/test_waypoint_file.py`: roundtrip save→load, duplicate-name rejection, bad `joint_names` rejection, reorder. No ROS, no robot.

**Acceptance:** `pytest`-free plain script runs green on the Mac; `colcon build` succeeds.

### Phase 1 — Teach node + step playback (robot, no torque release)

**Goal:** first milestone, end to end, using only interfaces that already exist today for motion.

1. `arm_teach.py`: subscribe `arm/joint_states`, publish `arm/joint_goals`, expose the 3 services, run the playback state machine, publish `arm/teach_status`.
2. `arm_teach.launch.py`: bridge + teach.
3. Arm is moved *by commanding it* (existing `arm/joint_goals` pub), then recorded. Manual hand-guiding comes in Phase 2.

**Test procedure (robot):**

```
ros2 launch sentinel_arm arm_teach.launch.py
ros2 topic pub arm/joint_goals sensor_msgs/msg/JointState \
  "{name: [base, shoulder, elbow, clamp], position: [0.1, 0.6, -0.4, 0.0]}"
# wait for motion to settle, then:
ros2 service call arm/record_waypoint sentinel_msgs/srv/RecordWaypoint "{name: 'pose1'}"
# ...repeat for pose2, pose3...
ros2 service call arm/waypoint_cmd sentinel_msgs/srv/WaypointCmd "{cmd: 'list'}"
ros2 service call arm/waypoint_cmd sentinel_msgs/srv/WaypointCmd "{cmd: 'save', arg: 'demo'}"
# restart the node (proves save/load), then:
ros2 service call arm/waypoint_cmd sentinel_msgs/srv/WaypointCmd "{cmd: 'load', arg: 'demo'}"
ros2 service call arm/playback sentinel_msgs/srv/Playback "{cmd: 'start', filename: 'demo'}"
ros2 topic echo arm/teach_status   # watch progress, exercise pause/resume/stop
```

**Acceptance:** 3 recorded poses replay; `arm/joint_states` converges to each within tolerance; pause holds, resume continues, stop freezes; saved file survives node restart. Also verify: the 4-joint `arm/joint_goals` publish (4× `T:101` back-to-back) produces simultaneous motion — see Risk #5.

### Phase 2 — Torque release: true manual teach (robot)

1. Bridge: add `arm/torque` service → `{"T":210,"cmd":0|1}`.
2. Teach node: `record_waypoint` while torque-off works (T:105 reads still valid — encoders, not motors); playback refuses to start while torque is off (first move command would re-lock mid-motion — see Risk #2).

**Acceptance:** torque off → arm hand-guidable; record 3 poses by hand; torque on; playback replays them.

### Phase 3 — Integration and polish

- Launch `arm_teach` from `sentinel_bringup.launch.py` (always-on with the arm node).
- `delay_after` honored in playback (implemented in Phase 1 code path, exercised here).
- Optional: per-waypoint `spd`/`acc` overrides — requires new bridge interface (e.g. `arm/move_joint` service carrying spd) or extended goal message. Only if gripper speed matters in practice.
- README update with teach/repeat quickstart.

**Acceptance:** full bringup boots arm + teach cleanly; a taught sequence replays from the normal bringup launch.

### Phase 4 — Stretch (only if needed)

- Joint-space interpolation mode (host-side ticker, ~20 Hz `T:101` along linear joint path).
- Record Cartesian `xyz` alongside joints (bridge must publish it — currently dropped).
- Export a waypoint file to the arm's onboard mission system (`T:220–224`) for standalone replay.
- Grasp gating with the gripper ADC sensors (`/gripper`) — stop closing clamp at a pressure threshold.
- Foxglove/RViz panel for teach/repeat.

---

## 5. Open Questions and Risks

1. **T:210 exact semantics unverified.** `cmd:0`/`cmd:1` payload shape and persistence are from the Waveshare wiki (web search; wiki itself 403'd for direct fetch). Verify empirically in Phase 2 before relying on it. Impact: manual teach blocked until verified. Mitigation: Phase 1 needs no T:210 at all.
2. **Torque re-locks on any motion command.** Per wiki, any rotation command re-enables torque. If playback starts while torque is off, the first `T:101` re-locks and moves the arm from wherever the operator left it — possibly a fast, unexpected jump. Mitigation: teach node tracks torque state and refuses playback while off; operator re-enables first.
3. **Per-joint T:101 timing.** 4 joints are commanded as 4 back-to-back `T:101`s today. Whether the arm starts all 4 simultaneously is unverified; if it serializes them, replay still works but looks different from the taught motion. Mitigation: Phase 1 test observes motion; if serialized and it matters, investigate the arm's multi-joint move command (`T:103`-style) as a bridge addition.
4. **Completion detection is heuristic.** "Reached" = 10 Hz joint states within tolerance for N consecutive samples. A slow settle or a drifting joint (torque-off sag) can false-positive. Defaults: tol 0.02 rad, 2 samples, 10 s timeout. Knobs are params. Impact: wrong "reached" verdict → next waypoint starts early. Mitigation: params + status log; tune on real hardware.
5. **Serial port is owned by the bridge.** Teach node must never open `/dev/sentinel_arm` (would reset the ESP32, 8 s boot drain, wedged bridge). Enforced by design: teach node contains no serial code. Risk: future contributors break this. Mitigation: docstring + README warning.
6. **Cartesian is dropped.** The bridge discards `x,y,z` from `T:1051`. Joint-space replay is exact, so nothing is lost for the milestone — but if the task later needs Cartesian moves (`T:1041`) or world-frame thinking (mobile base + arm), the bridge must start publishing EE pose. Flagged as Phase 4.
7. **`arm/waypoint_cmd` is stringly-typed.** One service with subcommands keeps v1 to 4 srv files, but typos surface at runtime. Upgrade path if this interface ever gets called from code that matters: split into typed services per operation.
8. **No base/arm coordination.** Teach-and-repeat assumes a stationary base; the drive stack runs independently. If a later milestone needs "drive to X then play sequence", that's a separate choreography node — out of scope, but the service interface (`arm/playback start`) is already the right primitive for it.
9. **Name collisions / file overwrite.** `save` overwrites silently (matches Unix convention); `record`/`rename` reject duplicates. If silent overwrite is unacceptable, add `--no-clobber`-style arg — trivial later.
10. **Waypoint files live only on the robot** (`~/arm_waypoints/`). Nothing syncs them to git automatically. For now, copy to the repo manually; the format is git-friendly. A sync step could join Phase 3 if sequences become numerous.
