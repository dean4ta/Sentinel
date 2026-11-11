#!/usr/bin/env python3
"""T004 — solve the rigid transform arm XYZ (mm) -> grid (mm).

Standalone:
    python3 solve_handeye.py pairs.json [out.yaml]

Reads pairs.json from handeye_capture.py (grid points in inches, arm
points in mm), converts the grid to mm, solves the least-squares rigid
transform  grid_mm = R @ arm_xyz_mm + t  with SVD/Kabsch Procrustes, and
writes handeye.yaml (R, t, rms_mm, per-point residuals, units,
timestamp, point count). Pure python3 + numpy: no ROS, no camera.

Exit codes:
  0  ok, handeye.yaml written
  1  fewer than 3 points in pairs.json
  2  bad args / missing or unreadable pairs.json
"""

import argparse
import json
import sys
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

IN_TO_MM = 25.4
MIN_POINTS = 3


def solve_rigid(src, dst):
    """Least-squares rigid transform: dst = R @ src + t (SVD Procrustes).

    src, dst: (N, 3) arrays, N >= 3 — e.g. arm XYZ mm -> grid mm.
    Returns (R (3,3), t (3,), rms, residuals (N,)) — residuals/rms in
    the same units as dst.
    """
    src = np.asarray(src, dtype=float)
    dst = np.asarray(dst, dtype=float)
    if src.ndim != 2 or dst.ndim != 2 or src.shape != dst.shape \
            or src.shape[1] != 3 or len(src) < MIN_POINTS:
        raise ValueError(f'solve_rigid needs >= {MIN_POINTS} matching '
                         f'(N,3) points, got {src.shape} / {dst.shape}')
    c_src = src.mean(axis=0)
    c_dst = dst.mean(axis=0)
    h = (src - c_src).T @ (dst - c_dst)
    u, _, vt = np.linalg.svd(h)
    r = vt.T @ u.T
    if np.linalg.det(r) < 0.0:  # reject the reflection solution
        vt[-1, :] *= -1.0
        r = vt.T @ u.T
    t = c_dst - r @ c_src
    residuals = np.linalg.norm((src @ r.T + t) - dst, axis=1)
    rms = float(np.sqrt(np.mean(residuals ** 2)))
    return r, t, rms, residuals


def arm_grid_arrays(points, unit_grid):
    """pairs 'points' list -> (arm (N,3) mm, grid (N,3) mm, labels (N,)).

    Grid points are 2D (board plane, inches): converted to mm and
    padded with z=0 in the grid frame. Raises ValueError on a bad unit
    or malformed point."""
    if unit_grid not in ('in', 'inch', 'inches'):
        raise ValueError(f"unit_grid must be inches, got {unit_grid!r}")
    try:
        arm = np.array([p['arm_xyz_mm'] for p in points], dtype=float)
        grid = np.array([p['grid'] for p in points], dtype=float) * IN_TO_MM
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f'malformed point in pairs.json: {exc}') from exc
    if arm.shape != (len(points), 3):
        raise ValueError('every point needs arm_xyz_mm [x, y, z] (mm)')
    if grid.shape != (len(points), 2):
        raise ValueError('every point needs grid [x_in, y_in]')
    grid = np.column_stack([grid, np.zeros(len(points))])
    labels = [str(p.get('target', i)) for i, p in enumerate(points)]
    return arm, grid, labels


def handeye_data(points, r, t, rms, residuals):
    """Result dict written to handeye.yaml (all lists, yaml-safe)."""
    return {
        'R': np.asarray(r, dtype=float).tolist(),
        't': np.asarray(t, dtype=float).tolist(),  # mm
        'rms_mm': float(rms),
        'per_point_residuals_mm': [float(v) for v in residuals],
        'units': {'arm': 'mm', 'grid': 'mm (converted from in)'},
        'equation': 'grid_mm = R @ arm_xyz_mm + t',
        'point_count': int(len(points)),
        'timestamp': datetime.now(timezone.utc).isoformat(
            timespec='seconds'),
    }


def plain_yaml(data):
    """Hand-written but valid YAML emitter (PyYAML-free fallback)."""
    lines = ['R:']
    for row in data['R']:
        lines.append('  - [' + ', '.join(repr(float(v)) for v in row) + ']')
    lines.append('t: [' + ', '.join(repr(float(v)) for v in data['t']) + ']')
    lines.append('rms_mm: ' + repr(float(data['rms_mm'])))
    lines.append('per_point_residuals_mm: ['
                 + ', '.join(repr(float(v))
                             for v in data['per_point_residuals_mm']) + ']')
    lines.append('units: {arm: mm, grid: '
                 + repr(data['units']['grid']) + '}')
    lines.append('equation: ' + repr(data['equation']))
    lines.append('point_count: ' + str(int(data['point_count'])))
    lines.append('timestamp: ' + repr(data['timestamp']))
    return '\n'.join(lines) + '\n'


def dump_yaml(out_path, data):
    """Write handeye.yaml. Prefers PyYAML; falls back to plain_yaml().

    Returns the writer name used (for the log line)."""
    try:
        import yaml  # noqa: F401
        text = yaml.safe_dump(data, default_flow_style=False,
                              sort_keys=False)
        writer = 'PyYAML'
    except ImportError:
        text = plain_yaml(data)
        writer = 'fallback text writer (PyYAML not installed)'
    Path(out_path).write_text(text)
    return writer


def print_solution(points, r, t, rms, residuals):
    """Print R (3x3), t (mm), per-point residuals (mm) and RMS."""
    print('R  (grid_mm = R @ arm_xyz_mm + t):')
    for row in np.asarray(r, dtype=float):
        print('    [' + ' '.join(f'{v:10.5f}' for v in row) + ' ]')
    print('t  (mm): [' + ' '.join(f'{v:.3f}' for v in np.asarray(t)) + ']')
    print('per-point residuals (mm):')
    for point, res in zip(points, residuals):
        print(f'    {str(point.get("target", "?")):26s} {res:7.3f}')
    print(f'RMS: {rms:.3f} mm')


def main(argv=None):
    ap = argparse.ArgumentParser(
        description='Solve arm->grid rigid transform from pairs.json '
                    '(T004).')
    ap.add_argument('pairs', help='pairs.json written by '
                                  'handeye_capture.py')
    ap.add_argument('out', nargs='?', default=None,
                    help='output yaml (default: handeye.yaml next to '
                         'pairs.json)')
    args = ap.parse_args(argv)

    pairs_path = Path(args.pairs)
    if not pairs_path.is_file():
        ap.error(f'pairs file not found: {pairs_path}')
    try:
        data = json.loads(pairs_path.read_text())
    except (OSError, json.JSONDecodeError) as exc:
        ap.error(f'cannot load {pairs_path}: {exc}')
    points = data.get('points') if isinstance(data, dict) else None
    if not isinstance(points, list) or len(points) < MIN_POINTS:
        n = len(points) if isinstance(points, list) else 0
        print(f'ERROR: need >= {MIN_POINTS} points in pairs.json, '
              f'got {n}', file=sys.stderr)
        return 1
    try:
        arm, grid, labels = arm_grid_arrays(points, data.get('unit_grid'))
    except ValueError as exc:
        ap.error(f'bad pairs.json: {exc}')
    if data.get('unit_arm') != 'mm':
        ap.error(f"unit_arm must be 'mm', got {data.get('unit_arm')!r}")
    try:
        r, t, rms, residuals = solve_rigid(arm, grid)
    except ValueError as exc:
        ap.error(str(exc))

    print(f'points: {len(points)} ({labels[0]} .. {labels[-1]})')
    print_solution(points, r, t, rms, residuals)
    out_path = Path(args.out) if args.out else \
        pairs_path.with_name('handeye.yaml')
    writer = dump_yaml(out_path, handeye_data(points, r, t, rms, residuals))
    print(f'wrote {out_path} ({writer})')
    return 0


if __name__ == '__main__':
    sys.exit(main())
