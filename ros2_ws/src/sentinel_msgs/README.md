# sentinel_msgs

Custom messages for Sentinel 1. ament_cmake (ament_python packages can't generate interfaces).

- `EncoderCounts` — raw 4x quadrature counts for FL/FR/BL/BR wheels. Published on `/encoder` by `sentinel_drive`'s `cmd_vel_serial`.
- `WheelSpeeds` — target, filtered and raw measured rpm, and commanded rpm per wheel (no Header; plain 16×float32). Published on `/wheel_speeds` by `cmd_vel_serial` (PID tuning data).
- `WheelConsistency` — `std_msgs/Header` + per-wheel confidence (0-1) and residual (m/s) from `sentinel_drive`'s `wheel_slip_detector`. Published on `/wheel_consistency`.
- `StepEvent` — start/end step timestamps from `sentinel_drive`'s `speed_step_test` (tuning only).
