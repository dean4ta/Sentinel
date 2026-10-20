#!/bin/bash
# run this from the project root directory
# Wheel speed-controller tuning run: all four wheels together through
# /cmd_vel vx steps, bagged and plotted. Robot on the floor; default
# steps travel ~10 m each way -- clear the path.
#
# Phase-agnostic: drive_params.yaml is the single source of truth.
#   FF phase:  pid_enabled: false, tune kS/kV
#   PID phase: pid_enabled: true (FF on), tune kp/ki
# The script runs whatever the yaml says; plots read the yaml too.
#
# Usage:
#   ./utils/wheel_tuning.sh
#   ./utils/wheel_tuning.sh "0.1,4;0.3,4;0,2;-0.1,4;-0.3,4"

STEPS="${1:-0.05,4;0.1,4;0.2,4;0.4,4;0.8,4;1.0,4;0,1;-0.05,4;-0.1,4;-0.2,4;-0.4,4;-0.8,4;-1.0,4}"

ssh sentinel1@ts_sentinel "
  source /opt/ros/humble/setup.bash && \
  cd ~/dean/Sentinel/ros2_ws && \
  colcon build && \
  source install/setup.bash && \
  cd .. && \
  ros2 launch sentinel_drive step_test.launch.py wheel:=ALL \
      steps:='$STEPS' sine:=none && \
  cd utils && \
  source .venv/bin/activate && \
  python analyze_step_test.py ../data/step_test_latest --wheel ALL \
      -o ../data && \
  deactivate
" && \
rm -rf data && mkdir -p data && \
scp 'sentinel1@ts_sentinel:/home/sentinel1/dean/Sentinel/data/*.png' \
  ~/UF/EEL4924_senior_design/project/data && \
open data/fl_step.png data/fr_step.png data/bl_step.png data/br_step.png
