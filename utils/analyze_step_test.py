#!/usr/bin/env python3
"""Extract /wheel_speeds from a rosbag and plot one wheel's step response.

Usage (from utils/, venv active):
  python analyze_step_test.py ../data/step_test_FL_2026-09-24 \
      --wheel FL -o ../data/fl_step.png

Uses the `rosbags` pip package; the sentinel_msgs/WheelSpeeds definition
is registered locally so no ROS install is needed.
"""

import argparse
import re
import sys
from pathlib import Path

from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

MSGTYPE = 'sentinel_msgs/msg/WheelSpeeds'

MSGDEF = (
    'float32 fl_target_rpm\n'
    'float32 fr_target_rpm\n'
    'float32 bl_target_rpm\n'
    'float32 br_target_rpm\n'
    'float32 fl_measured_rpm_filtered\n'
    'float32 fr_measured_rpm_filtered\n'
    'float32 bl_measured_rpm_filtered\n'
    'float32 br_measured_rpm_filtered\n'
    'float32 fl_measured_rpm_raw\n'
    'float32 fr_measured_rpm_raw\n'
    'float32 bl_measured_rpm_raw\n'
    'float32 br_measured_rpm_raw\n'
    'float32 fl_cmd_rpm\n'
    'float32 fr_cmd_rpm\n'
    'float32 bl_cmd_rpm\n'
    'float32 br_cmd_rpm\n'
)

TARGET_FIELDS = {'FL': 'fl_target_rpm', 'FR': 'fr_target_rpm',
                 'BL': 'bl_target_rpm', 'BR': 'br_target_rpm'}
MEASURED_FIELDS = {'FL': 'fl_measured_rpm_filtered',
                   'FR': 'fr_measured_rpm_filtered',
                   'BL': 'bl_measured_rpm_filtered',
                   'BR': 'br_measured_rpm_filtered'}
RAW_FIELDS = {'FL': 'fl_measured_rpm_raw', 'FR': 'fr_measured_rpm_raw',
              'BL': 'bl_measured_rpm_raw', 'BR': 'br_measured_rpm_raw'}
CMD_FIELDS = {'FL': 'fl_cmd_rpm', 'FR': 'fr_cmd_rpm',
              'BL': 'bl_cmd_rpm', 'BR': 'br_cmd_rpm'}
COUNT_FIELDS = {'FL': 'fl', 'FR': 'fr', 'BL': 'bl', 'BR': 'br'}

ENC_MSGTYPE = 'sentinel_msgs/msg/EncoderCounts'

ENC_MSGDEF = (
    'int32 fl\n'
    'int32 fr\n'
    'int32 bl\n'
    'int32 br\n'
)


def extract(bag_dir, wheel):
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    typestore.register(get_types_from_msg(MSGDEF, MSGTYPE))
    typestore.register(get_types_from_msg(ENC_MSGDEF, ENC_MSGTYPE))

    ts = []
    targets = []
    measured = []
    raw_meas = []
    cmds = []
    enc_ts = []
    counts = []
    with Reader(bag_dir) as reader:
        for conn, ts_, raw in reader.messages():
            if conn.msgtype == MSGTYPE:
                msg = typestore.deserialize_cdr(raw, conn.msgtype)
                ts.append(ts_)
                targets.append(getattr(msg, TARGET_FIELDS[wheel]))
                measured.append(getattr(msg, MEASURED_FIELDS[wheel]))
                raw_meas.append(getattr(msg, RAW_FIELDS[wheel]))
                cmds.append(getattr(msg, CMD_FIELDS[wheel]))
            elif conn.msgtype == ENC_MSGTYPE:
                msg = typestore.deserialize_cdr(raw, conn.msgtype)
                enc_ts.append(ts_)
                counts.append(getattr(msg, COUNT_FIELDS[wheel]))
    return ts, targets, measured, raw_meas, cmds, enc_ts, counts


def load_params():
    """Best-effort parse of drive_params.yaml for the plot subtitle."""
    path = Path(__file__).resolve().parent.parent / \
        'ros2_ws/src/sentinel_drive/config/drive_params.yaml'
    params = {}
    if not path.exists():
        return params
    for line in path.read_text().splitlines():
        m = re.match(r'^\s{4}(\w+):\s*([^#\s]+)', line)
        if m:
            params[m.group(1)] = m.group(2)
    return params


def plot(ts, targets, measured, raw_meas, cmds, enc_ts, counts, wheel,
         out_path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    RPM_TO_MPS = 0.44 / 60.0  # nominal 140 mm wheel circumference
    t0 = ts[0]
    t = [(x - t0) / 1e9 for x in ts]
    enc_t = [(x - t0) / 1e9 for x in enc_ts]
    targets = [v * RPM_TO_MPS for v in targets]
    cmds = [v * RPM_TO_MPS for v in cmds]
    measured = [v * RPM_TO_MPS for v in measured]
    raw_meas = [v * RPM_TO_MPS for v in raw_meas]

    fig, (ax_rpm, ax_cnt) = plt.subplots(
        2, 1, figsize=(12, 8), sharex=True)
    params = load_params()
    w = wheel.lower()
    if params.get('pid_enabled', 'true') == 'true':
        pid_txt = (f"kp={params.get(w + '_kp', '?')}, "
                   f"ki={params.get(w + '_ki', '?')}, "
                   f"kd={params.get(w + '_kd', '?')}")
    else:
        pid_txt = 'PID disabled'
    if params.get('feed_forward_enabled', 'true') == 'true':
        ff_txt = (f"kS={params.get(w + '_kS', '?')}, "
                  f"kV={params.get(w + '_kV', '?')}")
    else:
        ff_txt = 'FF disabled'
    fig.suptitle(
        f"{pid_txt}  |  {ff_txt}  |  "
        f"output_limit={params.get(w + '_output_limit', '?')}, "
        f"ema_alpha={params.get('rpm_ema_alpha', '?')}",
        fontsize=9)
    ax_rpm.plot(t, raw_meas, label='measured raw', color='tab:blue',
                alpha=0.12, lw=0.8)
    ax_rpm.plot(t, targets, label='target m/s', drawstyle='steps-post',
                alpha=0.5)
    ax_rpm.plot(t, cmds, label='commanded m/s', alpha=0.5)
    ax_rpm.plot(t, measured, label='measured m/s', alpha=0.8)
    ax_rpm.set_ylabel('m/s')
    ax_rpm.set_ylim(-2.5, 2.5)
    ax_rpm.set_title(f'{wheel} step response')
    ax_rpm.legend()
    ax_rpm.grid(True, alpha=0.3)

    ax_cnt.plot(enc_t, counts, label=f'{wheel} encoder counts',
                color='tab:green', alpha=0.8)
    ax_cnt.set_ylabel('ticks')
    ax_cnt.set_xlabel('time (s)')
    ax_cnt.legend()
    ax_cnt.grid(True, alpha=0.3)

    plt.tight_layout(rect=[0, 0, 1, 0.97])
    plt.savefig(out_path, dpi=150)
    print(f'wrote {out_path}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag_dir')
    ap.add_argument('--wheel', required=True, choices=sorted(TARGET_FIELDS))
    ap.add_argument('-o', '--out', required=True)
    args = ap.parse_args()

    ts, targets, measured, raw_meas, cmds, enc_ts, counts = extract(
        args.bag_dir, args.wheel)
    if not ts:
        print(f'no WheelSpeeds messages found in {args.bag_dir}', file=sys.stderr)
        raise SystemExit(1)
    plot(ts, targets, measured, raw_meas, cmds, enc_ts, counts, args.wheel,
         args.out)


if __name__ == '__main__':
    main()
