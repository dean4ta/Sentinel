#!/usr/bin/env python3
"""Overlay odometry bag trajectories for visual comparison.

Four variants (raw wheel odom / slip-compensated / raw+EKF /
slip+EKF) drawn as time-gradient XY paths over ONE shared time norm:
each path runs its own hue-family gradient (start -> end) while a
single colorbar maps time (s) for all four. Start ('o', black edge)
and end ('x') markers mark each path; the legend carries one entry
per variant plus the start/end marker key.

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
import numpy as np
from matplotlib.cm import ScalarMappable
from matplotlib.collections import LineCollection
from matplotlib.colors import LinearSegmentedColormap, Normalize

DEFAULT_OUT = ('/Users/dean/UF/EEL4924_senior_design/final_submission/'
               '02_wheel_odometry/odometry_comparison.png')

# Per-variant time-gradient pairs: start color -> end color, one hue
# family per variant so the four-path overlay stays distinguishable
# while still reading as a time gradient. Keyed by the replay bag dir
# names. Start colors are the dataviz categorical slots (blue/orange/
# aqua) plus red; ends are lighter steps of the same family. Identity
# is carried by the legend + start/end markers, never color alone.
GRADIENT = {
    'raw_odom': ('#2a78d6', '#7fd4e8'),   # blue -> light aqua
    'slip_odom': ('#eb6834', '#ffd27a'),  # orange -> light amber
    'raw_ekf': ('#1baf7a', '#b06ad0'),    # green -> purple
    'slip_ekf': ('#e34948', '#e87ba4'),   # red -> pink
}

FIG_DPI = 150


def _gradient_pairs(bags):
    """GRADIENT pair per bag: exact dir-name match first, then the
    remaining unused pairs in GRADIENT order (robust to custom names)."""
    pairs = [GRADIENT.get(Path(bag).name) for bag in bags]
    unused = iter(pair for pair in GRADIENT.values() if pair not in pairs)
    return [pair if pair is not None else next(unused) for pair in pairs]


def plot_compare(bags, out_png):
    """Draw all bag XY paths over one shared time norm; return
    {label: sample count} and print the per-bag summary."""
    # Collect trajectories first: the shared norm needs every path's
    # relative timestamps (each path's own first stamp is its t0).
    trajs = []
    for bag in bags:
        ts, xs, ys = extract_odom(bag)
        if not ts:
            print(f'no odometry in {bag}', file=sys.stderr)
            raise SystemExit(1)
        t = (np.asarray(ts, dtype=float) - ts[0]) / 1e9
        trajs.append((Path(bag).name, ts, t,
                      np.asarray(xs, dtype=float),
                      np.asarray(ys, dtype=float)))

    t_all = np.concatenate([t for _, _, t, _, _ in trajs])
    norm = Normalize(vmin=t_all.min(), vmax=max(t_all.max(), t_all.min() + 1e-9))

    fig, ax = plt.subplots(figsize=(8, 8))
    counts = {}
    legend_info = []
    for (label, ts, t, xs, ys), (start_c, end_c) in zip(
            trajs, _gradient_pairs(bags)):
        points = np.column_stack([xs, ys]).reshape(-1, 1, 2)
        segments = np.concatenate([points[:-1], points[1:]], axis=1)
        cmap = LinearSegmentedColormap.from_list(
            f'grad_{label}', [start_c, end_c])
        lc = LineCollection(segments, cmap=cmap, norm=norm, linewidth=2.0)
        lc.set_array(t)
        ax.add_collection(lc)
        ax.scatter(xs[0], ys[0], marker='o', s=70, facecolor=start_c,
                   edgecolor='black', linewidth=0.8, zorder=5)
        ax.scatter(xs[-1], ys[-1], marker='x', s=70, color=end_c,
                   linewidth=2.5, zorder=5)
        counts[label] = len(ts)
        legend_info.append((label, start_c))
        print(f'{label}: {len(ts)} samples, '
              f'end ({xs[-1]:.2f}, {ys[-1]:.2f})')

    # One shared colorbar: a neutral sequential mappable carrying the
    # shared norm (the four per-path colormaps cannot share one bar).
    sm = ScalarMappable(cmap='viridis', norm=norm)
    sm.set_array([])
    cbar = fig.colorbar(sm, ax=ax, pad=0.02)
    cbar.set_label('time (s)')

    ax.autoscale()
    ax.set_aspect('equal', adjustable='datalim')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title('Wheel odometry comparison: rectangle test\n'
                 '(raw x slip, with and without EKF; color = time)')
    ax.grid(True, alpha=0.3)

    handles = [plt.Line2D([0], [0], color=start_c, linewidth=2.0,
                          label=label)
               for label, start_c in legend_info]
    start_h = plt.Line2D([0], [0], marker='o', color='none',
                         markerfacecolor='white', markeredgecolor='black',
                         markersize=8, label='start')
    end_h = plt.Line2D([0], [0], marker='x', color='black',
                       markersize=9, label='end')
    ax.legend(handles=handles + [start_h, end_h], loc='best', fontsize=9)
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
