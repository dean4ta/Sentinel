# utils/hand_eye — T004 hand-eye touch calibration

Robot hand-guided to 12 grid targets; arm XYZ recorded at each; rigid
transform end-effector <-> grid solved with SVD Procrustes. See the
handoff at AGENT_WORKFLOW/handoffs/T004.md for the full run sequence,
target list and safety notes.

## Runtime dependencies (Orin)

No requirements.txt is needed: everything is either a ROS distro
package or one pip install.

- python3 with `numpy` and `opencv-python` (cv2)
- `dt-apriltags >= 3.1.7` — `pip3 install 'dt-apriltags>=3.1.7'`
- `PyYAML` — ROS distros ship it; the YAML writers fall back to a
  built-in plain emitter if it is missing
- ROS2 with `rclpy`, `geometry_msgs`, `sensor_msgs` and `sentinel_msgs`
  importable — `source ~/ros2_ws/install/setup.bash` before running
  `handeye_capture.py`
- A running arm bridge: `ros2 launch sentinel_arm arm_teach.launch.py`
  (provides `arm/ee_pose`, `arm/joint_states`, `arm/torque`)
- The C270 at `/dev/sentinel_webcam` (1280x720) and the T003
  `~/dean/camera_calibration/camera_matrix.yaml` (optional: missing
  file -> raw frames with a warning)

## Files

- `handeye_capture.py` — capture the 12 targets (ROS2 + camera);
  torque released on startup, ALWAYS re-locked on exit (SIGINT,
  SIGTERM and SIGHUP all take the re-lock path); writes `pairs.json`
  and auto-runs the solver -> `handeye.yaml`.
- `solve_handeye.py` — standalone: `python3 solve_handeye.py pairs.json
  [out.yaml]`. Core `solve_rigid(src, dst)` needs no ROS/camera.
- `test_handeye.py` — T006 verification (Orin, ROS2): visits the 12
  targets via `arm/ee_goals` (bridge T:1041 end-effector moves),
  default 10 mm lift above the board along the grid normal. Needs the
  bridge built with the T006 `arm/ee_goals` subscription.
- `selftest.py` — Mac-side checks, no ROS/camera/detector.
- `grid_5x4_tag36h11_ids18-37.json` — grid layout for target generation.

## Safety notes

- The capture tool never moves the arm and never opens the serial
  port.
- `test_handeye.py` (T006) DOES move the arm — its first run is the
  first autonomous arm motion. Keep clear of the arm and board; Q or
  Ctrl+C stops immediately (the arm holds its pose, torque stays
  locked — the script never releases torque). It stops after the
  first unreached target.
- `SIGKILL` cannot be caught: if the script is SIGKILLed while torque
  is released the arm stays limp — re-lock manually with
  `ros2 service call arm/torque sentinel_msgs/srv/SetTorque
  "{enable: true}"`.
