#!/usr/bin/env python3
"""Overlay two /wheel_odom bag trajectories for visual comparison.

Usage (from project root, utils venv active):
  python utils/plot_odom_compare.py data/replay/with_slip \
      data/replay/without_slip -o data/replay/compare.png
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from analyze_step_test import extract_odom  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag_with')
    ap.add_argument('bag_without')
    ap.add_argument('-o', '--out', default='odom_compare.png')
    args = ap.parse_args()

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(8, 8))
    for bag, label, ls, color in (
            (args.bag_with, 'with slip detector', '-', 'tab:blue'),
            (args.bag_without, 'without slip detector', '--', 'tab:red')):
        ts, xs, ys = extract_odom(bag)
        if not ts:
            print(f'no /wheel_odom in {bag}', file=sys.stderr)
            raise SystemExit(1)
        ax.plot(xs, ys, ls, color=color, label=label, lw=1.5)
        ax.plot(xs[0], ys[0], 'o', color=color, ms=6)
        ax.plot(xs[-1], ys[-1], 'x', color=color, ms=8, mew=2)
        print(f'{label}: {len(ts)} samples, '
              f'end ({xs[-1]:.2f}, {ys[-1]:.2f})')
    ax.set_aspect('equal')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title('wheel odometry: slip compensation on vs off')
    ax.legend()
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(args.out, dpi=150)
    print(f'wrote {args.out}')


if __name__ == '__main__':
    main()
