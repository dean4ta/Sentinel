#!/usr/bin/env python3
"""Overlay odometry bag trajectories for visual comparison.

Four variants (raw wheel odom / slip-compensated / raw+EKF /
slip+EKF) drawn as solid-color XY paths: one categorical color per
experiment, no time gradient. Start ('o', black edge) and end ('x')
markers mark each path; the legend carries one entry per variant plus
the start/end marker key.

Usage (from project root, utils venv active):
  python utils/plot_odom_compare.py data/replay/raw_odom \\
      data/replay/slip_odom data/replay/raw_ekf data/replay/slip_ekf

The default output lands in the final submission figures folder; pass
-o to override.
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from analyze_step_test import extract_odom  # noqa: E402

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

DEFAULT_OUT = ('/Users/dean/UF/EEL4924_senior_design/final_submission/'
               '02_wheel_odometry/odometry_comparison.png')

# Measured rectangle corners (user-measured feet -> meters):
# (0,0), (29 ft, 0), (29 ft, 15 ft), (0, 15 ft).
RECT_CORNERS_M = (
    (0.0, 0.0),
    (29 * 0.3048, 0.0),
    (29 * 0.3048, 15 * 0.3048),
    (0.0, 15 * 0.3048),
)

# One solid categorical color per variant, keyed by the replay bag dir
# names: dataviz categorical slots (blue/orange/aqua) plus red.
# Identity is carried by the legend + start/end markers, never color
# alone.
COLOR = {
    'raw_odom': '#2a78d6',   # blue
    'slip_odom': '#eb6834',  # orange
    'raw_ekf': '#1baf7a',    # green
    'slip_ekf': '#e34948',   # red
}

FIG_DPI = 150


def _colors(bags):
    """COLOR entry per bag: exact dir-name match first, then the
    remaining unused colors in COLOR order (robust to custom names)."""
    colors = [COLOR.get(Path(bag).name) for bag in bags]
    unused = iter(c for c in COLOR.values() if c not in colors)
    return [c if c is not None else next(unused) for c in colors]


def plot_compare(bags, out_png):
    """Draw all bag XY paths in one solid color each; return
    {label: sample count} and print the per-bag summary."""
    fig, ax = plt.subplots(figsize=(8, 8))
    counts = {}
    legend_info = []
    for bag, color in zip(bags, _colors(bags)):
        ts, xs, ys = extract_odom(bag)
        if not ts:
            print(f'no odometry in {bag}', file=sys.stderr)
            raise SystemExit(1)
        label = Path(bag).name
        ax.plot(xs, ys, color=color, linewidth=2.0)
        ax.scatter(xs[0], ys[0], marker='o', s=70, facecolor=color,
                   edgecolor='black', linewidth=0.8, zorder=5)
        ax.scatter(xs[-1], ys[-1], marker='x', s=70, color=color,
                   linewidth=2.5, zorder=5)
        counts[label] = len(ts)
        legend_info.append((label, color))
        print(f'{label}: {len(ts)} samples, '
              f'end ({xs[-1]:.2f}, {ys[-1]:.2f})')

    ax.set_aspect('equal', adjustable='datalim')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title('Wheel odometry comparison: rectangle test\n'
                 '(raw x slip, with and without EKF)')
    ax.grid(True, alpha=0.3)

    # Measured rectangle reference: dashed outline + corner dots.
    rect_x = [p[0] for p in RECT_CORNERS_M] + [RECT_CORNERS_M[0][0]]
    rect_y = [p[1] for p in RECT_CORNERS_M] + [RECT_CORNERS_M[0][1]]
    ax.plot(rect_x, rect_y, color='#52514e', linestyle='--',
            linewidth=1.2, label='measured rectangle')
    ax.scatter(rect_x[:-1], rect_y[:-1], marker='.', s=40,
               color='#52514e', zorder=4)

    handles = [plt.Line2D([0], [0], color=color, linewidth=2.0,
                          label=label)
               for label, color in legend_info]
    rect_h = plt.Line2D([0], [0], color='#52514e', linestyle='--',
                        linewidth=1.2, label='measured rectangle')
    start_h = plt.Line2D([0], [0], marker='o', color='none',
                         markerfacecolor='white', markeredgecolor='black',
                         markersize=8, label='start')
    end_h = plt.Line2D([0], [0], marker='x', color='black',
                       markersize=9, label='end')
    ax.legend(handles=handles + [rect_h, start_h, end_h],
              loc='best', fontsize=9)
    fig.tight_layout()

    out = Path(out_png)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=FIG_DPI)
    plt.close(fig)
    print(f'wrote {out}')
    return counts


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('bags', nargs='+')
    ap.add_argument('-o', '--out', default=DEFAULT_OUT)
    args = ap.parse_args(argv)
    return plot_compare(args.bags, args.out)


if __name__ == '__main__':
    main()
