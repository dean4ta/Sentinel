#!/usr/bin/env python3
"""Extract /wheel_speeds from a rosbag and plot one wheel's step response.

Usage (from utils/, venv active):
  python analyze_step_test.py ../data/step_test_FL_2026-09-24 \
      --wheel FL -o ../data/fl_step.png

Uses the `rosbags` pip package; the sentinel_msgs/WheelSpeeds definition
is registered locally so no ROS install is needed.
"""

import argparse
import sys

from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

MSGTYPE = 'sentinel_msgs/msg/WheelSpeeds'

MSGDEF = (
    'float32 fl_target_rpm\n'
    'float32 fr_target_rpm\n'
    'float32 bl_target_rpm\n'
    'float32 br_target_rpm\n'
    'float32 fl_measured_rpm\n'
    'float32 fr_measured_rpm\n'
    'float32 bl_measured_rpm\n'
    'float32 br_measured_rpm\n'
)

TARGET_FIELDS = {'FL': 'fl_target_rpm', 'FR': 'fr_target_rpm',
                 'BL': 'bl_target_rpm', 'BR': 'br_target_rpm'}
MEASURED_FIELDS = {'FL': 'fl_measured_rpm', 'FR': 'fr_measured_rpm',
                   'BL': 'bl_measured_rpm', 'BR': 'br_measured_rpm'}


def extract(bag_dir, wheel):
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    typestore.register(get_types_from_msg(MSGDEF, MSGTYPE))

    ts = []
    targets = []
    measured = []
    with Reader(bag_dir) as reader:
        for conn, ts_, raw in reader.messages():
            if conn.msgtype != MSGTYPE:
                continue
            msg = typestore.deserialize_ros2(raw, conn.msgtype)
            ts.append(ts_)
            targets.append(getattr(msg, TARGET_FIELDS[wheel]))
            measured.append(getattr(msg, MEASURED_FIELDS[wheel]))
    return ts, targets, measured


def plot(ts, targets, measured, wheel, out_path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    t0 = ts[0]
    t = [(x - t0) / 1e9 for x in ts]

    plt.figure(figsize=(12, 5))
    plt.plot(t, targets, label='target rpm', drawstyle='steps-post')
    plt.plot(t, measured, label='measured rpm', alpha=0.8)
    plt.xlabel('time (s)')
    plt.ylabel('rpm')
    plt.title(f'{wheel} step response')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(out_path, dpi=150)
    print(f'wrote {out_path}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag_dir')
    ap.add_argument('--wheel', required=True, choices=sorted(TARGET_FIELDS))
    ap.add_argument('-o', '--out', required=True)
    args = ap.parse_args()

    ts, targets, measured = extract(args.bag_dir, args.wheel)
    if not ts:
        print(f'no WheelSpeeds messages found in {args.bag_dir}', file=sys.stderr)
        raise SystemExit(1)
    plot(ts, targets, measured, args.wheel, args.out)


if __name__ == '__main__':
    main()
