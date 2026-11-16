#!/usr/bin/env python3
"""Selftest for plot_odom_compare.py.

Synthesizes four minimal rosbag2 dirs in a temp dir (raw / slip /
raw+EKF / slip+EKF variants: axis-aligned rectangle paths with
progressively shifted copies, header stamps increasing), then runs
the real plotter on them. Plain asserts, no pytest, no robot.

Run: utils/.venv/bin/python3 utils/test_plot_odom_compare.py
"""

import ast
import contextlib
import io
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import numpy as np
from rosbags.rosbag2 import Writer
from rosbags.typesys import Stores, get_typestore

import plot_odom_compare as poc

TYPESTORE = get_typestore(Stores.ROS2_HUMBLE)
ODOM_MSGTYPE = 'nav_msgs/msg/Odometry'
N_SAMPLES = 100
DT_NS = 50_000_000          # 20 Hz stamps
T0_NS = 1_700_000_000_000_000_000

# (bag dir name, x shift, y shift, rectangle scale) -- progressively
# shifted copies so the overlay stays separable.
VARIANTS = (
    ('raw_odom', 0.0, 0.0, 1.00),
    ('slip_odom', 0.4, 0.3, 1.05),
    ('raw_ekf', 0.8, 0.6, 1.10),
    ('slip_ekf', 1.2, 0.9, 1.15),
)


def odom_msg(t_ns, x, y):
    """One Odometry message at stamp t_ns (ns), position (x, y)."""
    T = TYPESTORE.types
    cov = np.zeros(36)
    return T[ODOM_MSGTYPE](
        header=T['std_msgs/msg/Header'](
            stamp=T['builtin_interfaces/msg/Time'](
                sec=t_ns // 1_000_000_000, nanosec=t_ns % 1_000_000_000),
            frame_id='odom'),
        child_frame_id='base_link',
        pose=T['geometry_msgs/msg/PoseWithCovariance'](
            pose=T['geometry_msgs/msg/Pose'](
                position=T['geometry_msgs/msg/Point'](x=x, y=y, z=0.0),
                orientation=T['geometry_msgs/msg/Quaternion'](
                    x=0.0, y=0.0, z=0.0, w=1.0)),
            covariance=cov.copy()),
        twist=T['geometry_msgs/msg/TwistWithCovariance'](
            twist=T['geometry_msgs/msg/Twist'](
                linear=T['geometry_msgs/msg/Vector3'](x=0.0, y=0.0, z=0.0),
                angular=T['geometry_msgs/msg/Vector3'](x=0.0, y=0.0, z=0.0)),
            covariance=cov.copy()))


def rectangle_path(shift_x, shift_y, scale, t0_ns, n=N_SAMPLES):
    """Axis-aligned rectangle (2.0 x 1.0 m scaled), n samples clockwise
    from the origin, timestamps increasing by DT_NS from t0_ns."""
    ts, xs, ys = [], [], []
    per_side = n // 4
    w, h = 2.0 * scale, 1.0 * scale
    for i in range(n):
        side, f = divmod(i, per_side)
        f = f / max(per_side - 1, 1)
        if side == 0:
            x, y = w * f, 0.0
        elif side == 1:
            x, y = w, h * f
        elif side == 2:
            x, y = w * (1.0 - f), h
        else:
            x, y = 0.0, h * (1.0 - f)
        ts.append(t0_ns + i * DT_NS)
        xs.append(x + shift_x)
        ys.append(y + shift_y)
    return ts, xs, ys


def write_bag(path, pts):
    """Write pts (ts, xs, ys lists) as /wheel_odom messages to a bag."""
    ts, xs, ys = pts
    with Writer(path, version=8) as writer:
        conn = writer.add_connection('/wheel_odom', ODOM_MSGTYPE,
                                     typestore=TYPESTORE)
        for t_ns, x, y in zip(ts, xs, ys):
            writer.write(conn, t_ns, TYPESTORE.serialize_cdr(
                odom_msg(t_ns, x, y), ODOM_MSGTYPE))


def test_gradient():
    """GRADIENT: four entries, distinct start colors, distinct end
    colors, start != end within each pair."""
    assert len(poc.GRADIENT) == 4, poc.GRADIENT
    pairs = list(poc.GRADIENT.values())
    starts = [p[0] for p in pairs]
    ends = [p[1] for p in pairs]
    assert len(set(starts)) == 4, f'duplicate start colors: {starts}'
    assert len(set(ends)) == 4, f'duplicate end colors: {ends}'
    for start_c, end_c in pairs:
        assert start_c != end_c, (start_c, end_c)
    print('ok: GRADIENT has 4 entries with distinct start/end colors')


def test_py310_grammar():
    """plot_odom_compare.py must parse as Python 3.10 grammar."""
    src = (HERE / 'plot_odom_compare.py').read_text()
    ast.parse(src, filename='plot_odom_compare.py', feature_version=(3, 10))
    print('ok: plot_odom_compare.py parses as py310 grammar')


def test_plot():
    """Four synthesized bags -> main() -> PNG exists/non-empty and the
    returned + printed per-bag sample counts match."""
    with tempfile.TemporaryDirectory(prefix='t013_odom_selftest_') as tmp:
        tmp = Path(tmp)
        bags = []
        for idx, (name, sx, sy, scale) in enumerate(VARIANTS):
            # Each bag starts at its own epoch so stamps differ across
            # bags (each path's own first stamp becomes its t0).
            path = tmp / name
            write_bag(path, rectangle_path(sx, sy, scale,
                                           T0_NS + idx * 60_000_000_000))
            bags.append(str(path))

        out_png = tmp / 'odometry_comparison.png'
        printed = io.StringIO()
        with contextlib.redirect_stdout(printed):
            counts = poc.main(bags + ['-o', str(out_png)])
        text = printed.getvalue()

        assert out_png.exists(), f'missing output: {out_png}'
        size = out_png.stat().st_size
        assert size > 10_000, f'PNG suspiciously small: {size} bytes'
        assert set(counts) == {name for name, _, _, _ in VARIANTS}, counts
        for name, n in counts.items():
            assert n == N_SAMPLES, (name, n)
            assert f'{name}: {n} samples' in text, \
                f'missing printed summary for {name}'
        assert f'wrote {out_png}' in text, 'missing "wrote" line'
        print(f'ok: {len(VARIANTS)} bags x {N_SAMPLES} msgs -> '
              f'{out_png.name} ({size} bytes)')


if __name__ == '__main__':
    test_gradient()
    test_py310_grammar()
    test_plot()
    print('selftest passed')
