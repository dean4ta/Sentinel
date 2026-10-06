#!/bin/bash
# run this from the project root directory
# Offline odometry-measurement replay. Plays the input bag ONCE on the
# Orin while four namespaced odometry pipelines run in parallel:
#   pass 1: raw wheel odom         -> /pass1/wheel_odom
#   pass 2: slip-compensated odom  -> /pass2/wheel_odom
#   pass 3: raw wheel odom + EKF   -> /pass3/wheel_imu_odom_filtered
#   pass 4: slip-compensated + EKF -> /pass4/wheel_imu_odom_filtered
# Then pulls the four result bags back and plots them overlaid.
# Wall time = one bag duration (vs 4x for sequential passes); CPU at
# rate 1.0 fits the Orin Nano's 6 cores.
#
# Usage: ./utils/odometry_measurements_replay.sh [input_bag] [rate]
#   input_bag: bag dir on the Orin, default data/odom_input
#   rate: rosbag play rate, default 1.0 (raise for long routes; watch CPU)

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

  # Stop all pipeline nodes + recorders when the play finishes.
  # ros2 CLI wrappers don't always forward signals to their children,
  # so pkill by cmdline pattern.
  finish() {
    pkill -INT -f 'wheel_odometry|wheel_slip_detector|ekf_nod[e]|ros2 bag record' 2>/dev/null
    sleep 3   # rosbag finalizes metadata.yaml on SIGINT
    pkill -9 -f 'wheel_odometry|wheel_slip_detector|ekf_nod[e]|ros2 bag record' 2>/dev/null
    wait 2>/dev/null
  }

  # $1 = pass number. wheel_odometry against the shared /wheel_speeds;
  # own odom + consistency topics keep the four pipelines isolated.
  # Consistency topic without a publisher (passes 1/3) = equal weights.
  run_odom() {
    ros2 run sentinel_drive wheel_odometry --ros-args \
      -p use_sim_time:=true \
      -r /wheel_odom:=/pass$1/wheel_odom \
      -r /wheel_consistency:=/pass$1/wheel_consistency \
      </dev/null >/dev/null 2>&1 &
  }

  # $1 = pass number, $2 = odom topic to fuse.
  # imu0 (gyro) comes from the bag, shared by both EKF instances.
  # publish_tf off: two EKFs would fight over odom -> base_link.
  run_ekf() {
    ros2 run robot_localization ekf_node --ros-args \
      --params-file "$EKF_CFG" \
      -p use_sim_time:=true \
      -p publish_tf:=false \
      -p odom0:=$2 \
      -r odometry/filtered:=/pass$1/wheel_imu_odom_filtered \
      </dev/null >/dev/null 2>&1 &
  }

  echo "replay: 4 pipelines, one pass (rate $RATE)"
  ros2 bag play "$IN" -r "$RATE" --clock \
    --topics /wheel_speeds /camera/realsense2_camera/gyro/sample /tf_static \
    </dev/null >/dev/null 2>&1 & PLAY=$!
  sleep 3

  # pass 1: raw wheel odom
  run_odom 1
  ros2 bag record /pass1/wheel_odom -o data/replay/raw_odom \
    </dev/null >/dev/null 2>&1 &

  # pass 2: slip-compensated odom
  ros2 run sentinel_drive wheel_slip_detector --ros-args \
    -p use_sim_time:=true \
    -r /wheel_consistency:=/pass2/wheel_consistency \
    </dev/null >/dev/null 2>&1 &
  run_odom 2
  ros2 bag record /pass2/wheel_odom -o data/replay/slip_odom \
    </dev/null >/dev/null 2>&1 &

  # pass 3: raw odom + EKF
  run_odom 3
  run_ekf 3 /pass3/wheel_odom
  ros2 bag record /pass3/wheel_imu_odom_filtered -o data/replay/raw_ekf \
    </dev/null >/dev/null 2>&1 &

  # pass 4: slip-compensated odom + EKF
  ros2 run sentinel_drive wheel_slip_detector --ros-args \
    -p use_sim_time:=true \
    -r /wheel_consistency:=/pass4/wheel_consistency \
    </dev/null >/dev/null 2>&1 &
  run_odom 4
  run_ekf 4 /pass4/wheel_odom
  ros2 bag record /pass4/wheel_imu_odom_filtered -o data/replay/slip_ekf \
    </dev/null >/dev/null 2>&1 &

  ticker $PLAY &
  wait $PLAY
  echo "  play finished, stopping pipelines"
  finish

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
