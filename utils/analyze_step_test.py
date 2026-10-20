#!/usr/bin/env python3
"""Extract /wheel_speeds from a rosbag and plot one wheel's step response.

Usage (from utils/, venv active):
  python analyze_step_test.py ../data/step_test_FL_2026-09-24 \
      --wheel FL -o ../data/fl_step.png
  python analyze_step_test.py ../data/step_test_latest \
      --wheel ALL -o ../data   # writes {fl,fr,bl,br}_step.png

Also handles gamepad.launch.py bags (no /step_events): the per-step
encoder-average overlay is skipped automatically when no step events
are present.

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

STEP_MSGTYPE = 'sentinel_msgs/msg/StepEvent'

STEP_MSGDEF = (
    'int32 index\n'
    'float32 target_mps\n'
    'bool sine\n'
)

WC_MSGTYPE = 'sentinel_msgs/msg/WheelConsistency'

WC_MSGDEF = (
    'std_msgs/Header header\n'
    'float32 fl_confidence\n'
    'float32 fr_confidence\n'
    'float32 bl_confidence\n'
    'float32 br_confidence\n'
    'float32 fl_residual\n'
    'float32 fr_residual\n'
    'float32 bl_residual\n'
    'float32 br_residual\n'
)

ODOM_MSGTYPE = 'nav_msgs/msg/Odometry'  # built into the ROS2 typestore

# Encoder count sign during forward motion (mirrored mounting).
ENC_DIR = {'FL': -1.0, 'FR': 1.0, 'BL': -1.0, 'BR': 1.0}


def compute_step_averages(events, enc_ts, counts, wheel):
    """Average m/s per step from encoder count deltas between step
    start/end timestamps. Skips the sine phase. Returns
    [(t_start, t_end, avg_mps)] in ns timestamps."""
    import bisect
    avgs = []
    step_events = [e for e in events if not e[3]]
    n = len(step_events)
    if n == 0 or not enc_ts:
        return avgs
    for i, (t0, idx, mps, sine) in enumerate(step_events):
        if i + 1 < n:
            t1 = step_events[i + 1][0]
        else:
            # Last step: ends at the final encoder sample (no end event).
            t1 = enc_ts[-1]
        # Trim 10 samples (~200 ms) from each side: accel/decel transients
        # at step edges stay out of the average.
        j0 = bisect.bisect_left(enc_ts, t0) + 10
        j1 = bisect.bisect_left(enc_ts, t1) - 1 - 10
        if j1 <= j0:
            continue
        dt = (enc_ts[j1] - enc_ts[j0]) / 1e9
        if dt <= 0:
            continue
        dcounts = counts[j1] - counts[j0]
        avg = dcounts / 532.0 * 0.44 / dt * ENC_DIR[wheel]
        avgs.append((t0, t1, avg))
    return avgs


def extract(bag_dir, wheel):
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    typestore.register(get_types_from_msg(MSGDEF, MSGTYPE))
    typestore.register(get_types_from_msg(ENC_MSGDEF, ENC_MSGTYPE))
    typestore.register(get_types_from_msg(STEP_MSGDEF, STEP_MSGTYPE))

    ts = []
    targets = []
    measured = []
    raw_meas = []
    cmds = []
    enc_ts = []
    counts = []
    events = []
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
            elif conn.msgtype == STEP_MSGTYPE:
                msg = typestore.deserialize_cdr(raw, conn.msgtype)
                events.append(
                    (ts_, msg.index, msg.target_mps, msg.sine))
    return ts, targets, measured, raw_meas, cmds, enc_ts, counts, events


def load_params():
    """Best-effort parse of drive_params.yaml for the plot subtitle."""
    path = Path(__file__).resolve().parent.parent / \
        'ros2_ws/src/sentinel_bringup/config/drive_params.yaml'
    params = {}
    if not path.exists():
        return params
    for line in path.read_text().splitlines():
        m = re.match(r'^\s{4}(\w+):\s*([^#\s]+)', line)
        if m:
            params[m.group(1)] = m.group(2)
    return params


def plot(ts, targets, measured, raw_meas, cmds, enc_ts, counts, events,
         wheel, out_path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    RPM_TO_MPS = 0.44 / 60.0  # nominal 140 mm wheel circumference
    t0 = ts[0]
    t = [(x - t0) / 1e9 for x in ts]
    enc_t = [(x - t0) / 1e9 for x in enc_ts]
    t_base = t0
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
        f"ema_alpha={params.get(w + '_ema_alpha', '?')}",
        fontsize=9)
    ax_rpm.plot(t, raw_meas, label='measured raw', color='tab:blue',
                alpha=0.12, lw=0.8)
    ax_rpm.plot(t, targets, label='target m/s', drawstyle='steps-post',
                alpha=0.5)
    ax_rpm.plot(t, cmds, label='commanded m/s', alpha=0.5)
    ax_rpm.plot(t, measured, label='measured m/s', alpha=0.8)
    ax_rpm.set_ylabel('m/s')
    ax_rpm.set_ylim(-2.5, 2.5)
    ax_rpm.set_title(f'{wheel} response')
    ax_rpm.legend()
    ax_rpm.grid(True, alpha=0.3)

    ax_cnt.plot(enc_t, counts, label=f'{wheel} encoder counts',
                color='tab:green', alpha=0.8)
    ax_cnt.set_ylabel('ticks')
    ax_cnt.set_xlabel('time (s)')
    ax_cnt.legend(loc='upper left')
    ax_cnt.grid(True, alpha=0.3)

    # Encoder-derived average speed per step, on a secondary axis.
    avgs = compute_step_averages(events, enc_ts, counts, wheel)
    if avgs:
        ax_avg = ax_cnt.twinx()
        for i, (t0, t1, avg) in enumerate(avgs):
            x0 = (t0 - t_base) / 1e9
            x1 = (t1 - t_base) / 1e9
            ax_avg.axvline(x0, color='tab:red', ls=':', alpha=0.5, lw=1)
            ax_avg.axvline(x1, color='tab:red', ls=':', alpha=0.5, lw=1)
            ax_avg.plot([x0, x1], [avg, avg], color='tab:red', alpha=0.7,
                        lw=1.5,
                        label='avg m/s (encoder)' if i == 0 else None)
            ax_avg.text((x0 + x1) / 2, avg, f'{avg:.2f}',
                        ha='center', va='bottom', fontsize=8,
                        color='tab:red', alpha=0.9)
        ax_avg.set_ylabel('avg m/s (encoder)')
        ax_avg.set_ylim(-2.5, 2.5)
        ax_avg.legend(loc='upper right')
        ax_avg.grid(False)

    plt.tight_layout(rect=[0, 0, 1, 0.97])
    plt.savefig(out_path, dpi=150)
    print(f'wrote {out_path}')


def extract_consistency(bag_dir):
    """/wheel_consistency -> (ts, {wheel: confidence}, {wheel: residual}).
    Empty lists if the topic is absent (step-test bags)."""
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    typestore.register(get_types_from_msg(WC_MSGDEF, WC_MSGTYPE))
    ts = []
    confs = {w: [] for w in TARGET_FIELDS}
    resids = {w: [] for w in TARGET_FIELDS}
    with Reader(bag_dir) as reader:
        for conn, ts_, raw in reader.messages():
            if conn.msgtype != WC_MSGTYPE:
                continue
            msg = typestore.deserialize_cdr(raw, conn.msgtype)
            ts.append(ts_)
            for w in TARGET_FIELDS:
                confs[w].append(getattr(msg, w.lower() + '_confidence'))
                resids[w].append(getattr(msg, w.lower() + '_residual'))
    return ts, confs, resids


def plot_consistency(ts, confs, resids, out_path):
    """All four wheels: confidence (0-1) + residual (m/s), one figure."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    t0 = ts[0]
    t = [(x - t0) / 1e9 for x in ts]
    params = load_params()
    fig, (ax_conf, ax_res) = plt.subplots(
        2, 1, figsize=(12, 8), sharex=True)
    fig.suptitle(
        "wheel slip detector: threshold="
        f"{params.get('residual_threshold_mps', '?')}, scale="
        f"{params.get('residual_scale_mps', '?')}, tracking_scale="
        f"{params.get('tracking_scale_mps', '?')}, margin="
        f"{params.get('peer_margin_mps', '?')}, alpha="
        f"{params.get('alpha_suspicious', '?')}/"
        f"{params.get('alpha_recover', '?')}",
        fontsize=9)
    for w in TARGET_FIELDS:
        ax_conf.plot(t, confs[w], label=w, lw=1.5)
        ax_res.plot(t, resids[w], label=w, lw=0.8, alpha=0.8)
    ax_conf.set_ylabel('confidence')
    ax_conf.set_ylim(-0.05, 1.05)
    ax_conf.set_title('wheel consistency (1.0 = trusted)')
    ax_conf.legend()
    ax_conf.grid(True, alpha=0.3)
    ax_res.set_ylabel('residual m/s')
    ax_res.set_xlabel('time (s)')
    ax_res.legend()
    ax_res.grid(True, alpha=0.3)
    plt.tight_layout(rect=[0, 0, 1, 0.97])
    plt.savefig(out_path, dpi=150)
    print(f'wrote {out_path}')


def extract_odom(bag_dir):
    """/wheel_odom -> (ts, xs, ys). Empty if the topic is absent."""
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    ts = []
    xs = []
    ys = []
    with Reader(bag_dir) as reader:
        for conn, ts_, raw in reader.messages():
            if conn.msgtype != ODOM_MSGTYPE:
                continue
            msg = typestore.deserialize_cdr(raw, conn.msgtype)
            ts.append(ts_)
            xs.append(msg.pose.pose.position.x)
            ys.append(msg.pose.pose.position.y)
    return ts, xs, ys


def plot_odom(ts, xs, ys, out_path):
    """2D trajectory, line colored from start (viridis start) to end."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection
    import numpy as np

    t0 = ts[0]
    t = np.array([(x - t0) / 1e9 for x in ts])
    pts = np.array([xs, ys]).T.reshape(-1, 1, 2)
    segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
    lc = LineCollection(segs, cmap='viridis', norm=plt.Normalize(t[0], t[-1]))
    lc.set_array(t[:-1])
    lc.set_linewidth(1.5)

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.add_collection(lc)
    ax.autoscale()
    ax.set_aspect('equal')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title(f'wheel odometry path ({t[-1]:.1f} s)')
    ax.grid(True, alpha=0.3)
    fig.colorbar(lc, ax=ax, label='time (s)')
    plt.tight_layout()
    plt.savefig(out_path, dpi=150)
    print(f'wrote {out_path}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag_dir')
    ap.add_argument('--wheel', required=True,
                    choices=sorted(TARGET_FIELDS) + ['ALL'])
    ap.add_argument('-o', '--out', required=True,
                    help='single-wheel: output .png path. ALL: output '
                         'directory, one {fl,fr,bl,br}_step.png each')
    args = ap.parse_args()

    if args.wheel == 'ALL':
        for w in ('FL', 'FR', 'BL', 'BR'):
            out = str(Path(args.out) / f'{w.lower()}_step.png')
            ts, targets, measured, raw_meas, cmds, enc_ts, counts, events = \
                extract(args.bag_dir, w)
            if not ts:
                print(f'no WheelSpeeds messages found in {args.bag_dir}',
                      file=sys.stderr)
                raise SystemExit(1)
            plot(ts, targets, measured, raw_meas, cmds, enc_ts, counts,
                 events, w, out)
    else:
        ts, targets, measured, raw_meas, cmds, enc_ts, counts, events = \
            extract(args.bag_dir, args.wheel)
        if not ts:
            print(f'no WheelSpeeds messages found in {args.bag_dir}',
                  file=sys.stderr)
            raise SystemExit(1)
        plot(ts, targets, measured, raw_meas, cmds, enc_ts, counts, events,
             args.wheel, args.out)

    # Optional: /wheel_consistency figure alongside the step plots,
    # skipped silently when the bag has no such topic.
    cts, confs, resids = extract_consistency(args.bag_dir)
    if cts:
        out = str(Path(args.out if args.wheel == 'ALL'
                       else Path(args.out).parent) / 'consistency.png')
        plot_consistency(cts, confs, resids, out)

    # Optional: /wheel_odom 2D trajectory, same skip rules.
    ots, oxs, oys = extract_odom(args.bag_dir)
    if ots:
        out = str(Path(args.out if args.wheel == 'ALL'
                       else Path(args.out).parent) / 'odometry.png')
        plot_odom(ots, oxs, oys, out)


if __name__ == '__main__':
    main()
