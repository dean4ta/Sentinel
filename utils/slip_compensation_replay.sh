#!/bin/bash
# run this from the project root directory
# Offline slip-compensation comparison. Replays the input bag twice on
# the Orin, both times running wheel_odometry against the replayed
# /wheel_speeds and recording its /wheel_odom output:
#   pass 1: with wheel_slip_detector live (confidences weight the solve)
#   pass 2: without it (odometry falls back to equal weights)
# Then pulls both result bags back and plots them overlaid.
#
# Usage: ./utils/slip_compensation_replay.sh [input_bag] [rate]
#   input_bag: bag dir on the Orin, default data/slip_input
#   rate: rosbag play rate, default 1.0 (raise for long routes)

IN="${1:-data/slip_input}"
RATE="${2:-1.0}"

ssh sentinel1@ts_sentinel IN="$IN" RATE="$RATE" 'bash -s' <<'REMOTE'
  source /opt/ros/humble/setup.bash
  cd ~/dean/Sentinel
  source ros2_ws/install/setup.bash
  rm -rf data/replay && mkdir -p data/replay

  # pass 1: with slip detector
  ros2 bag play "$IN" -r "$RATE" --clock >/dev/null 2>&1 & PLAY=$!
  sleep 3
  ros2 run sentinel_drive wheel_slip_detector \
    --ros-args -p use_sim_time:=true >/dev/null 2>&1 & SLIP=$!
  ros2 run sentinel_drive wheel_odometry \
    --ros-args -p use_sim_time:=true >/dev/null 2>&1 & ODOM=$!
  ros2 bag record /wheel_odom -o data/replay/with_slip >/dev/null 2>&1 & REC=$!
  wait $PLAY
  sleep 2
  kill $REC $ODOM $SLIP 2>/dev/null; wait 2>/dev/null

  # pass 2: without slip detector (no /wheel_consistency -> equal weights)
  ros2 bag play "$IN" -r "$RATE" --clock >/dev/null 2>&1 & PLAY=$!
  sleep 3
  ros2 run sentinel_drive wheel_odometry \
    --ros-args -p use_sim_time:=true >/dev/null 2>&1 & ODOM=$!
  ros2 bag record /wheel_odom -o data/replay/without_slip >/dev/null 2>&1 & REC=$!
  wait $PLAY
  sleep 2
  kill $REC $ODOM 2>/dev/null; wait 2>/dev/null
  echo "replay done"
REMOTE

rm -rf data/replay
scp -r sentinel1@sentinel1-desktop.tail919736.ts.net:/home/sentinel1/dean/Sentinel/data/replay \
  ~/UF/EEL4924_senior_design/project/data && \
utils/.venv/bin/python3 utils/plot_odom_compare.py \
  data/replay/with_slip data/replay/without_slip \
  -o data/replay/compare.png && \
open data/replay/compare.png
