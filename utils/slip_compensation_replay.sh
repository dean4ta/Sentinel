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
#   input_bag: bag dir on the Orin, default data/gamepad_latest (so a
#              ground_drive_tuning.sh recording replays as-is; only
#              /wheel_speeds is played back, the recorded derived
#              topics are filtered out)
#   rate: rosbag play rate, default 1.0 (raise for long routes)

IN="${1:-data/gamepad_latest}"
RATE="${2:-1.0}"

ssh sentinel1@ts_sentinel IN="$IN" RATE="$RATE" 'bash -s' <<'REMOTE'
  source /opt/ros/humble/setup.bash
  cd ~/dean/Sentinel
  source ros2_ws/install/setup.bash
  # Kill strays from an interrupted previous run (they'd publish
  # /wheel_odom into the new recordings).
  pkill -9 -f 'wheel_odometry|wheel_slip_detector|ros2 bag' 2>/dev/null
  sleep 1
  rm -rf data/replay && mkdir -p data/replay

  # Bag duration (s) for the progress ticker. Empty string disables it.
  DUR=$(ros2 bag info "$IN" 2>/dev/null | sed -n 's/^Duration: *\([0-9.]*\)s.*/\1/p')

  # Progress ticker: % played = wall time * rate / bag duration.
  # Needs its own stdin: bag play reads stdin for keyboard controls,
  # so every background ROS process gets </dev/null.
  ticker() {  # $1 = play pid
    T0=$SECONDS
    while kill -0 "$1" 2>/dev/null; do
      if [ -n "$DUR" ]; then
        awk -v e=$((SECONDS - T0)) -v r="$RATE" -v d="$DUR" \
          'BEGIN{printf "\r  %.0f%% of bag played   ", (e*r/d)*100}'
      fi
      sleep 5
    done
    [ -n "$DUR" ] && printf '\r  100%% of bag played\n'
  }

  # Stop nodes + recorder for one pass. ros2 CLI wrappers don't always
  # forward signals to their children, so pkill by cmdline pattern.
  finish_pass() {
    pkill -INT -f 'wheel_odometry|wheel_slip_detector|ros2 bag record' 2>/dev/null
    sleep 3   # rosbag finalizes metadata.yaml on SIGINT
    pkill -9 -f 'wheel_odometry|wheel_slip_detector|ros2 bag record' 2>/dev/null
    wait 2>/dev/null
  }

  # pass 1: with slip detector
  echo "replay pass 1/2: with slip detector (rate $RATE)"
  ros2 bag play "$IN" -r "$RATE" --clock --topics /wheel_speeds \
    </dev/null >/dev/null 2>&1 & PLAY=$!
  sleep 3
  ros2 run sentinel_drive wheel_slip_detector \
    --ros-args -p use_sim_time:=true </dev/null >/dev/null 2>&1 &
  ros2 run sentinel_drive wheel_odometry \
    --ros-args -p use_sim_time:=true </dev/null >/dev/null 2>&1 &
  ros2 bag record /wheel_odom -o data/replay/with_slip \
    </dev/null >/dev/null 2>&1 &
  ticker $PLAY &
  wait $PLAY
  echo "  play finished, stopping nodes"
  finish_pass

  # pass 2: without slip detector (no /wheel_consistency -> equal weights)
  echo "replay pass 2/2: without slip detector (rate $RATE)"
  ros2 bag play "$IN" -r "$RATE" --clock --topics /wheel_speeds \
    </dev/null >/dev/null 2>&1 & PLAY=$!
  sleep 3
  ros2 run sentinel_drive wheel_odometry \
    --ros-args -p use_sim_time:=true </dev/null >/dev/null 2>&1 &
  ros2 bag record /wheel_odom -o data/replay/without_slip \
    </dev/null >/dev/null 2>&1 &
  ticker $PLAY &
  wait $PLAY
  echo "  play finished, stopping nodes"
  finish_pass

  test -f data/replay/with_slip/metadata.yaml || \
    { echo "ERROR: with_slip bag not finalized"; exit 1; }
  test -f data/replay/without_slip/metadata.yaml || \
    { echo "ERROR: without_slip bag not finalized"; exit 1; }
  echo "replay done"
REMOTE

rm -rf data/replay
scp -r sentinel1@sentinel1-desktop.tail919736.ts.net:/home/sentinel1/dean/Sentinel/data/replay \
  ~/UF/EEL4924_senior_design/project/data && \
utils/.venv/bin/python3 utils/plot_odom_compare.py \
  data/replay/with_slip data/replay/without_slip \
  -o data/replay/compare.png && \
open data/replay/compare.png
