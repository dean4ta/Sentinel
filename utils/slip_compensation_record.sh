#!/bin/bash
# run this from the project root directory
# Slip-compensation input recording: drive a quick route while recording
# ONLY /wheel_speeds and /encoder (the derived topics stay out, so the
# bag can be replayed offline with and without the slip detector).
# Press X (cross) on the PS4 controller to end the run.
#
# Usage: ./utils/slip_compensation_record.sh

ssh sentinel1@ts_sentinel '
  source /opt/ros/humble/setup.bash && \
  cd ~/dean/Sentinel/ && \
  rm -rf data/slip_input && \
  cd ros2_ws/ && \
  colcon build && \
  source install/setup.bash && \
  cd .. && \
  ros2 launch sentinel_drive gamepad.launch.py record:=true \
    bag_out:=data/slip_input \
    record_topics:="/wheel_speeds /encoder"
'
