# Per-wheel speed controller state: target rpm, measured rpm (EMA-
# filtered and raw), and commanded rpm (PID+ff output x WHEEL_MAX_RPM)
# for all four wheels. Published by sentinel_drive's cmd_vel_serial at
# 50 Hz on /wheel_speeds. Used for PID tuning (recorded to rosbag).
float32 fl_target_rpm
float32 fr_target_rpm
float32 bl_target_rpm
float32 br_target_rpm
float32 fl_measured_rpm_filtered
float32 fr_measured_rpm_filtered
float32 bl_measured_rpm_filtered
float32 br_measured_rpm_filtered
float32 fl_measured_rpm_raw
float32 fr_measured_rpm_raw
float32 bl_measured_rpm_raw
float32 br_measured_rpm_raw
float32 fl_cmd_rpm
float32 fr_cmd_rpm
float32 bl_cmd_rpm
float32 br_cmd_rpm
