# Per-wheel consistency from wheel_slip_detector (leave-one-out
# mecanum check). confidence: 1.0 = fully trusted, 0.0 = suspicious.
# residual: measured - predicted wheel surface speed (m/s), signed;
# suspicion math in wheel_consistency.py projects it onto each wheel's
# direction of travel. Header lets consumers time-align with
# /wheel_speeds or encoder counts.
std_msgs/Header header
float32 fl_confidence
float32 fr_confidence
float32 bl_confidence
float32 br_confidence
float32 fl_residual
float32 fr_residual
float32 bl_residual
float32 br_residual
