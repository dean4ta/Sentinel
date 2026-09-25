# sentinel_msgs

Custom messages for Sentinel 1. ament_cmake (ament_python packages can't generate interfaces).

- `EncoderCounts` — raw 4x quadrature counts for FL/FR/BL/BR wheels, published on `/encoder` by `sentinel_drive`'s `cmd_vel_serial`.
- `WheelSpeeds` — target + measured rpm for FL/FR/BL/BR, published on `/wheel_speeds` by `sentinel_drive`'s `cmd_vel_serial` (PID tuning data).
