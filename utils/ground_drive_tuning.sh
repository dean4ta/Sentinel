#!/bin/bash
# run this from the project root directory
# Ground drive tuning run: gamepad drive stack + rosbag recording.
# Press X (cross) on the PS4 controller to end the run: the launch shuts
# down, the bag closes, plots are generated and opened.

ssh sentinel1@ts_sentinel '
  source /opt/ros/humble/setup.bash && \
  cd ~/dean/Sentinel/ && \
  rm -rf data/ && \
  cd ros2_ws/ && \
  colcon build && \
  source install/setup.bash && \
  cd .. && \
  ros2 launch sentinel_drive_cpp gamepad.launch.py record:=true && \
  cd utils/ && \
  source .venv/bin/activate && \
  python analyze_step_test.py ../data/gamepad_latest --wheel ALL -o ../data && \
  deactivate && \
  cd ..
' && \
rm -rf data && \
scp -r sentinel1@sentinel1-desktop.tail919736.ts.net:/home/sentinel1/dean/Sentinel/data \
  ~/UF/EEL4924_senior_design/project && \
open data/fl_step.png data/fr_step.png data/bl_step.png data/br_step.png \
     data/consistency.png data/odometry.png
