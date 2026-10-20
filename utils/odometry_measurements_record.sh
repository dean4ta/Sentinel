#!/bin/bash
# run this from the project root directory
# Odometry-measurement recording: drive a route while recording ONLY the
# raw inputs - /wheel_speeds, /encoder, the D435i gyro, and /tf_static
# (camera extrinsics the replay EKF needs). Derived topics stay out, so
# the bag replays offline under any odometry pipeline.
# Press X (cross) on the PS4 controller to end the run.
#
# Usage: ./utils/odometry_measurements_record.sh

ssh sentinel1@ts_sentinel '
  source /opt/ros/humble/setup.bash && \
  cd ~/dean/Sentinel/ && \
  rm -rf data/odom_input && \
  cd ros2_ws/ && \
  colcon build && \
  source install/setup.bash && \
  source ~/dean/rs2_ws/install/setup.bash && \
  cd ..
  ros2 launch sentinel_bringup fusion.launch.py >/tmp/fusion_record.log 2>&1 &
  FUSION=$!
  sleep 12
  ros2 launch sentinel_drive_cpp gamepad.launch.py record:=true \
    bag_out:=data/odom_input \
    record_topics:="/wheel_speeds /encoder /camera/realsense2_camera/gyro/sample /tf_static"
  RC=$?
  kill -INT $FUSION 2>/dev/null
  sleep 2
  pkill -INT -f "realsense2_camera_nod[e]"
  pkill -INT -f "ekf_nod[e]"
  pkill -INT -f "static_transform_publishe[r]"
  exit $RC
'
