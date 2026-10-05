#!/bin/bash
# run this from the project root directory
# Offline odometry-measurement replay. Replays the input bag four times
# on the Orin, running the odometry pipeline against replayed inputs and
# recording its output:
#   pass 1: raw wheel odom         -> /wheel_odom
#   pass 2: slip-compensated odom  -> /wheel_odom
#   pass 3: raw wheel odom + EKF   -> /wheel_imu_odom_filtered
#   pass 4: slip-compensated + EKF -> /wheel_imu_odom_filtered
# Then pulls the four result bags back and plots them overlaid.
#
# Usage: ./utils/odometry_measurements_replay.sh [input_bag] [rate]
#   input_bag: bag dir on the Orin, default data/odom_input
#   rate: rosbag play rate, default 1.0 (raise for long routes)

IN="${1:-data/odom_input}"
RATE="${2:-1.0}"

ssh sentinel1@ts_sentinel IN="$IN" RATE="$RATE" 'bash -s' <<'REMOTE'
  source /opt/ros/humble/setup.bash
  cd ~/dean/Sentinel
  source ros2_ws/install/setup.bash
  EKF_CFG=ros2_ws/install/sentinel_drive/share/sentinel_drive/config/ekf.yaml
  # Kill strays from an interrupted previous run (they'd publish
  # odometry into the new recordings).
  pkill -9 -f 'wheel_odometry|wheel_slip_detector|ekf_nod[e]|ros2 bag' 2>/dev/null
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
    pkill -INT -f 'wheel_odometry|wheel_slip_detector|ekf_nod[e]|ros2 bag record' 2>/dev/null
    sleep 3   # rosbag finalizes metadata.yaml on SIGINT
    pkill -9 -f 'wheel_odometry|wheel_slip_detector|ekf_nod[e]|ros2 bag record' 2>/dev/null
    wait 2>/dev/null
  }

  # $1 = bag name, $2 = slip detector (on/off), $3 = EKF (on/off),
  # $4 = topic to record. /tf_static from the bag supplies the EKF's
  # base_link -> camera_link -> camera_gyro_optical_frame chain.
  run_pass() {
    echo "replay pass: $1 (rate $RATE)"
    ros2 bag play "$IN" -r "$RATE" --clock \
      --topics /wheel_speeds /camera/realsense2_camera/gyro/sample /tf_static \
      </dev/null >/dev/null 2>&1 & PLAY=$!
    sleep 3
    if [ "$2" = on ]; then
      ros2 run sentinel_drive wheel_slip_detector \
        --ros-args -p use_sim_time:=true </dev/null >/dev/null 2>&1 &
    fi
    ros2 run sentinel_drive wheel_odometry \
      --ros-args -p use_sim_time:=true </dev/null >/dev/null 2>&1 &
    if [ "$3" = on ]; then
      ros2 run robot_localization ekf_node --ros-args \
        --params-file "$EKF_CFG" \
        -r odometry/filtered:=/wheel_imu_odom_filtered \
        -p use_sim_time:=true </dev/null >/dev/null 2>&1 &
    fi
    ros2 bag record "$4" -o "data/replay/$1" \
      </dev/null >/dev/null 2>&1 &
    ticker $PLAY &
    wait $PLAY
    echo "  play finished, stopping nodes"
    finish_pass
  }

  run_pass raw_odom  off off /wheel_odom
  run_pass slip_odom on  off /wheel_odom
  run_pass raw_ekf   off on  /wheel_imu_odom_filtered
  run_pass slip_ekf  on  on  /wheel_imu_odom_filtered

  for b in raw_odom slip_odom raw_ekf slip_ekf; do
    test -f "data/replay/$b/metadata.yaml" || \
      { echo "ERROR: $b bag not finalized"; exit 1; }
  done
  echo "replay done"
REMOTE

rm -rf data/replay
scp -r sentinel1@sentinel1-desktop.tail919736.ts.net:/home/sentinel1/dean/Sentinel/data/replay \
  ~/UF/EEL4924_senior_design/project/data && \
utils/.venv/bin/python3 utils/plot_odom_compare.py \
  data/replay/raw_odom data/replay/slip_odom data/replay/raw_ekf data/replay/slip_ekf \
  -o data/replay/compare.png && \
open data/replay/compare.png
