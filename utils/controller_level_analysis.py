#!/usr/bin/env python3
# T012 Mac-side analysis: publication plots for
# final_submission/00_wheel_speed_controller/.
#
# Reads the CSVs recorded on the robot by sentinel_debug's step_recorder
# (one dir per controller level: wheel_speeds.csv, wheel_odom.csv,
# wheel_imu_odom_filtered.csv, step_events.csv) and writes the PNG set:
#   01_no_controllers_wheels.png / _odom.png   (level off)
#   02_feedforward_wheels.png    / _odom.png   (level ff)
#   03_ff_pid_wheels.png         / _odom.png   (level ffpid)
#   04_combined_wheels.png       / _odom.png   (FL + XY, 3 levels)
#
# Odometry source per level: /wheel_imu_odom_filtered (wheel odom + EKF,
# our best method) when its CSV exists, else /wheel_odom -- labeled on
# each plot.
#
# Run (Mac, no ROS):
#   uv run --with numpy --with matplotlib python3 \
#       utils/controller_level_analysis.py
#
# Pure parsing/selection logic above the `import numpy` seam so the
# selftest (ros2_ws/src/sentinel_debug/test/test_step_recorder.py) can
# AST-extract and exercise it with the stdlib only.

import argparse
import csv
import pathlib

LEVELS = ('off', 'ff', 'ffpid')
LEVEL_STEMS = {
    'off': '01_no_controllers',
    'ff': '02_feedforward',
    'ffpid': '03_ff_pid',
}
LEVEL_LABELS = {
    'off': 'no controllers (open loop)',
    'ff': 'feed-forward only',
    'ffpid': 'feed-forward + PID + EMA',
}
WHEELS = ('fl', 'fr', 'bl', 'br')
WHEEL_LABELS = {
    'fl': 'FL (front-left)',
    'fr': 'FR (front-right)',
    'bl': 'BL (back-left)',
    'br': 'BR (back-right)',
}

# Column names MUST match sentinel_debug/step_recorder.py exactly -- the
# selftest cross-checks the two lists (robot/Mac contract).
WHEEL_SPEEDS_FIELDS = (
    'fl_target_rpm', 'fr_target_rpm', 'bl_target_rpm', 'br_target_rpm',
    'fl_measured_rpm_filtered', 'fr_measured_rpm_filtered',
    'bl_measured_rpm_filtered', 'br_measured_rpm_filtered',
    'fl_measured_rpm_raw', 'fr_measured_rpm_raw',
    'bl_measured_rpm_raw', 'br_measured_rpm_raw',
    'fl_cmd_rpm', 'fr_cmd_rpm', 'bl_cmd_rpm', 'br_cmd_rpm')
ODOM_FIELDS = ('x', 'y', 'yaw', 'vx', 'wz')
STEP_EVENT_FIELDS = ('index', 'target_mps', 'sine')

WHEEL_SPEEDS_CSV = 'wheel_speeds.csv'
WHEEL_ODOM_CSV = 'wheel_odom.csv'
FILTERED_ODOM_CSV = 'wheel_imu_odom_filtered.csv'
STEP_EVENTS_CSV = 'step_events.csv'


def find_level_dir(root, level):
    """Data dir for a controller level under root; accepts both copy
    conventions the handoff documents: level_off and
    controller_level_off (the recorder's on-robot name)."""
    root = pathlib.Path(root)
    for name in (f'level_{level}', f'controller_level_{level}'):
        candidate = root / name
        if candidate.is_dir():
            return candidate
    return None


def choose_odom_csv(level_dir):
    """Best odometry CSV in a level dir: the EKF output
    (wheel_imu_odom_filtered) when present, else wheel_odom, else None."""
    filtered = pathlib.Path(level_dir) / FILTERED_ODOM_CSV
    if filtered.is_file():
        return filtered
    wheel = pathlib.Path(level_dir) / WHEEL_ODOM_CSV
    if wheel.is_file():
        return wheel
    return None


def odom_source_label(csv_path):
    """Plot label for which odometry source a CSV came from."""
    if FILTERED_ODOM_CSV in str(csv_path):
        return 'wheel_imu_odom_filtered (wheel odom + EKF)'
    return 'wheel_odom (wheel odom only)'


def parse_csv(path):
    """CSV -> {column: [float, ...]}; the first column is `t`."""
    columns = {}
    with open(path, newline='', encoding='utf-8') as f:
        for line in csv.reader(f):
            if not columns:
                columns = {name: [] for name in line}
                continue
            for name, value in zip(columns, line):
                columns[name].append(float(value))
    return columns


def wheel_series(data):
    """{t, <w>_target, <w>_measured} short names from a parsed
    wheel_speeds.csv (target = fl_target_rpm, measured = the EMA
    filtered rpm -- the msg's target vs measured fields)."""
    out = {'t': data['t']}
    for wheel in WHEELS:
        out[wheel + '_target'] = data[wheel + '_target_rpm']
        out[wheel + '_measured'] = data[wheel + '_measured_rpm_filtered']
    return out


# ---- plotting below (imports numpy + matplotlib) ----

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.colors import LinearSegmentedColormap, Normalize

# Fixed categorical colors: dataviz reference palette slots 1-3
# (blue/orange/aqua), validated all-pairs on a light surface; the
# legends carry the identity, so color is never the only cue.
COLOR = {'off': '#2a78d6', 'ff': '#eb6834', 'ffpid': '#1baf7a'}
TARGET_COLOR = '#52514e'   # target reference: dashed dark gray
EVENT_COLOR = '#8a8884'    # step-event vlines: recessive gray

# Per-level time-gradient pairs for the XY trajectories: start color ->
# end color, one hue family per level so the 3-level overlay stays
# distinguishable while still reading as a time gradient.
GRADIENT = {
    'off': ('#2a78d6', '#7fd4e8'),    # blue -> light aqua
    'ff': ('#eb6834', '#ffd27a'),     # orange -> light amber
    'ffpid': ('#1baf7a', '#b06ad0'),  # green -> purple
}

FIG_DPI = 150


def _rel(data, t0):
    """Array of values relative to a reference time (epoch -> 0)."""
    return np.asarray(data, dtype=float) - t0


def plot_level_wheels(level, wheels, steps, out_png):
    """4-panel (FL/FR/BL/BR) target vs measured wheel speed over time,
    step events as vertical lines."""
    t0 = wheels['t'][0]
    t = _rel(wheels['t'], t0)
    fig, axes = plt.subplots(2, 2, figsize=(12, 7.5), sharex=True)
    fig.suptitle(
        f'Wheel step response -- {LEVEL_LABELS[level]}', fontsize=13)
    for ax, wheel in zip(axes.flat, WHEELS):
        ax.plot(t, wheels[wheel + '_target'],
                color=TARGET_COLOR, linestyle='--', linewidth=1.5,
                label='target')
        ax.plot(t, wheels[wheel + '_measured'],
                color=COLOR[level], linewidth=2.0, label='measured (EMA)')
        for event_t in _rel(steps['t'], t0):
            ax.axvline(event_t, color=EVENT_COLOR, linewidth=0.8,
                       linestyle=':', label='step event' if wheel == 'fl'
                       else None)
        ax.set_title(WHEEL_LABELS[wheel])
        ax.set_xlabel('time (s)')
        ax.set_ylabel('wheel speed (rpm)')
        ax.grid(alpha=0.3, linewidth=0.5)
    axes.flat[0].legend(loc='best', fontsize=9)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(out_png, dpi=FIG_DPI)
    plt.close(fig)


def _gradient_xy(ax, odom, level, label):
    """XY trajectory as a time-gradient line (start color -> end color),
    with explicit start/end markers. Returns the LineCollection so the
    caller can attach a colorbar."""
    t = _rel(odom['t'], odom['t'][0])
    points = np.column_stack([odom['x'], odom['y']]).reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)
    cmap = LinearSegmentedColormap.from_list(f'grad_{level}', GRADIENT[level])
    norm = Normalize(vmin=t.min(), vmax=max(t.max(), t.min() + 1e-9))
    lc = LineCollection(segments, cmap=cmap, norm=norm, linewidth=2.0,
                        label=label)
    lc.set_array(t)
    ax.add_collection(lc)
    start_c, end_c = GRADIENT[level]
    ax.scatter(odom['x'][0], odom['y'][0], marker='o', s=70,
               facecolor=start_c, edgecolor='black', linewidth=0.8,
               zorder=5)
    ax.scatter(odom['x'][-1], odom['y'][-1], marker='x', s=70,
               color=end_c, linewidth=2.5, zorder=5)
    return lc, norm, cmap


def plot_level_odom(level, odom, source_label, out_png):
    """XY trajectory (time-gradient) + x/y vs time subplots for one level."""
    t0 = odom['t'][0]
    t = _rel(odom['t'], t0)
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.5))
    fig.suptitle(
        f'Odometry -- {LEVEL_LABELS[level]}\nsource: {source_label}',
        fontsize=12)
    lc, norm, cmap = _gradient_xy(axes[0], odom, level, None)
    axes[0].set_title('XY trajectory (color = time)')
    axes[0].set_xlabel('x (m)')
    axes[0].set_ylabel('y (m)')
    axes[0].set_aspect('equal', adjustable='datalim')
    axes[0].grid(alpha=0.3, linewidth=0.5)
    cbar = fig.colorbar(lc, ax=axes[0], pad=0.02)
    cbar.set_label('time (s)')
    axes[1].plot(t, odom['x'], color=COLOR[level], linewidth=2.0)
    axes[1].set_title('x vs time')
    axes[1].set_xlabel('time (s)')
    axes[1].set_ylabel('x (m)')
    axes[1].grid(alpha=0.3, linewidth=0.5)
    axes[2].plot(t, odom['y'], color=COLOR[level], linewidth=2.0)
    axes[2].set_title('y vs time')
    axes[2].set_xlabel('time (s)')
    axes[2].set_ylabel('y (m)')
    axes[2].grid(alpha=0.3, linewidth=0.5)
    fig.tight_layout(rect=(0, 0, 1, 0.90))
    fig.savefig(out_png, dpi=FIG_DPI)
    plt.close(fig)


def plot_combined_wheels(level_data, out_png):
    """FL wheel, three controller levels overlaid (target dashed)."""
    fig, ax = plt.subplots(figsize=(9, 5))
    steps = None
    steps_t0 = None
    for level in LEVELS:
        if level not in level_data:
            continue
        wheels = level_data[level]['wheels']
        t = _rel(wheels['t'], wheels['t'][0])
        ax.plot(t, wheels['fl_measured'], color=COLOR[level],
                linewidth=2.0, label=LEVEL_LABELS[level])
        if steps is None and level_data[level].get('steps'):
            # One run's target trace stands for all (same steps command).
            ax.plot(t, wheels['fl_target'], color=TARGET_COLOR,
                    linestyle='--', linewidth=1.5, label='target')
            steps = level_data[level]['steps']
            steps_t0 = wheels['t'][0]  # vlines share the trace's t0
    if steps and steps['t']:
        first = True
        for event_t in _rel(steps['t'], steps_t0):
            ax.axvline(event_t, color=EVENT_COLOR, linewidth=0.8,
                       linestyle=':', label='step event' if first else None)
            first = False
    ax.set_title('FL step response: three controller levels')
    ax.set_xlabel('time (s)')
    ax.set_ylabel('wheel speed (rpm)')
    ax.grid(alpha=0.3, linewidth=0.5)
    ax.legend(loc='best', fontsize=9)
    fig.tight_layout()
    fig.savefig(out_png, dpi=FIG_DPI)
    plt.close(fig)


def plot_combined_odom(level_data, out_png):
    """XY trajectory, three controller levels overlaid. Each trajectory
    carries its own hue-family time gradient (start -> end) plus o/x
    start/end markers; a legend entry per level keeps the three
    gradients separable without one shared colorbar (three per-trace
    colorbars would fight)."""
    fig, ax = plt.subplots(figsize=(7.5, 6))
    plotted = []
    for level in LEVELS:
        if level not in level_data or level_data[level]['odom'] is None:
            continue
        odom = level_data[level]['odom']
        lc, _norm, _cmap = _gradient_xy(
            ax, odom, level,
            label=f"{LEVEL_LABELS[level]} -- "
                  f"{level_data[level]['odom_csv'].stem}")
        plotted.append((level, lc))
    ax.set_title('Odometry XY: three controller levels\n'
                 '(color = time along each trajectory: start -> end)')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_aspect('equal', adjustable='datalim')
    ax.grid(alpha=0.3, linewidth=0.5)
    handles = [plt.Line2D([0], [0], color=GRADIENT[lv][0], linewidth=2.0,
                          label=f'{LEVEL_LABELS[lv]}')
               for lv, _lc in plotted]
    start_h = plt.Line2D([0], [0], marker='o', color='none',
                         markerfacecolor='white', markeredgecolor='black',
                         markersize=8, label='start')
    end_h = plt.Line2D([0], [0], marker='x', color='black',
                       markersize=9, label='end')
    ax.legend(handles=handles + [start_h, end_h], loc='best', fontsize=9)
    fig.tight_layout()
    fig.savefig(out_png, dpi=FIG_DPI)
    plt.close(fig)


def plot_combined_xy_time(level_data, out_png):
    """X vs time and Y vs time, three controller levels overlaid per
    panel. Solid per-level categorical colors (time is already the x
    axis, so a gradient would double-encode and muddy the overlay)."""
    fig, axes = plt.subplots(1, 2, figsize=(14, 4.5))
    fig.suptitle('Odometry components vs time: three controller levels',
                 fontsize=12)
    for level in LEVELS:
        if level not in level_data or level_data[level]['odom'] is None:
            continue
        odom = level_data[level]['odom']
        t = _rel(odom['t'], odom['t'][0])
        label = (f"{LEVEL_LABELS[level]} -- "
                 f"{level_data[level]['odom_csv'].stem}")
        axes[0].plot(t, odom['x'], color=COLOR[level], linewidth=2.0,
                     label=label)
        axes[1].plot(t, odom['y'], color=COLOR[level], linewidth=2.0,
                     label=label)
    for ax, name in zip(axes, ('x (m)', 'y (m)')):
        ax.set_title(f'{name} vs time')
        ax.set_xlabel('time (s)')
        ax.set_ylabel(name)
        ax.grid(alpha=0.3, linewidth=0.5)
        ax.legend(loc='best', fontsize=9)
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    fig.savefig(out_png, dpi=FIG_DPI)
    plt.close(fig)


def load_level(level_dir):
    """{wheels, steps, odom, odom_csv} for one level dir; missing pieces
    become None instead of aborting the run."""
    data = {}
    wheels_path = pathlib.Path(level_dir) / WHEEL_SPEEDS_CSV
    if wheels_path.is_file():
        data['wheels'] = wheel_series(parse_csv(wheels_path))
    else:
        data['wheels'] = None
    steps_path = pathlib.Path(level_dir) / STEP_EVENTS_CSV
    data['steps'] = parse_csv(steps_path) if steps_path.is_file() else None
    odom_csv = choose_odom_csv(level_dir)
    data['odom'] = parse_csv(odom_csv) if odom_csv else None
    data['odom_csv'] = odom_csv
    return data


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='T012 controller-level step test plots (Mac, no ROS)')
    parser.add_argument(
        '--input',
        default='/Users/dean/UF/EEL4924_senior_design/final_submission/'
                '00_wheel_speed_controller/data',
        help='dir with level_off/level_ff/level_ffpid subdirs '
             '(controller_level_* names also accepted)')
    parser.add_argument(
        '--output',
        default='/Users/dean/UF/EEL4924_senior_design/final_submission/'
                '00_wheel_speed_controller',
        help='dir the PNGs are written to')
    args = parser.parse_args(argv)

    root = pathlib.Path(args.input)
    out_dir = pathlib.Path(args.output)
    out_dir.mkdir(parents=True, exist_ok=True)

    level_data = {}
    for level in LEVELS:
        level_dir = find_level_dir(root, level)
        if level_dir is None:
            print(f'skip {level}: no data dir under {root} '
                  f'(looked for level_{level}/ and '
                  f'controller_level_{level}/)')
            continue
        level_data[level] = load_level(level_dir)

    for level in LEVELS:
        if level not in level_data:
            continue
        stem = LEVEL_STEMS[level]
        wheels = level_data[level]['wheels']
        steps = level_data[level]['steps']
        if not wheels or not wheels['t']:
            print(f'skip {level}: wheel_speeds.csv missing or empty')
            continue
        steps = steps if steps else {'t': [], 'target_mps': []}
        plot_level_wheels(level, wheels, steps,
                          out_dir / f'{stem}_wheels.png')
        print(f'wrote {out_dir / (stem + "_wheels.png")}')
        odom = level_data[level]['odom']
        if odom is not None and odom['t']:
            plot_level_odom(
                level, odom,
                odom_source_label(level_data[level]['odom_csv']),
                out_dir / f'{stem}_odom.png')
            print(f'wrote {out_dir / (stem + "_odom.png")} '
                  f'(source: {level_data[level]["odom_csv"].name})')
        else:
            print(f'skip {level} odometry: neither {WHEEL_ODOM_CSV} nor '
                  f'{FILTERED_ODOM_CSV} in {find_level_dir(root, level)}')

    if any(level_data.get(level, {}).get('wheels') and
           level_data[level]['wheels']['t'] for level in LEVELS):
        plot_combined_wheels(level_data,
                             out_dir / '04_combined_wheels.png')
        print(f'wrote {out_dir / "04_combined_wheels.png"}')
    if any(level_data.get(level, {}).get('odom') and
           level_data[level]['odom']['t'] for level in LEVELS):
        plot_combined_odom(level_data, out_dir / '04_combined_odom.png')
        print(f'wrote {out_dir / "04_combined_odom.png"}')
    if any(level_data.get(level, {}).get('odom') and
           level_data[level]['odom']['t'] for level in LEVELS):
        plot_combined_xy_time(level_data,
                              out_dir / '05_combined_xy_time.png')
        print(f'wrote {out_dir / "05_combined_xy_time.png"}')


if __name__ == '__main__':
    main()
