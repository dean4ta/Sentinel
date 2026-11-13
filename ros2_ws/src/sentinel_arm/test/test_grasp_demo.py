#!/usr/bin/env python3
"""T007 selftest — Mac-side, no ROS, no camera, no detector.

Exercises the importable core of grasp_demo.py (the ROS node class is
guarded behind the rclpy import, so this runs in plain Python):

  1. Target-square indexing against the real grid JSON: 12 squares,
     row-major; (row 2, col 1) = squares[7] = (3.7333, 5.6) in = the
     "3rd row, between 2nd and 3rd columns" square; out-of-range raises
  2. Hand-eye transform round-trip (grid -> arm -> grid exact) and the
     z-lift math: target + hover_z along the signed normal changes only
     the grid-normal component by exactly hover_z; lift_direction
     normalizes, flips a through-board R and keeps an away-from-board R
  3. clamp deg -> rad (10 deg = 0.17453292519943295 rad)
  4. Sequence harness, all actions failing: all 7 stages still run in
     order and the run COMPLETES; each failed action logs a WARN naming
     the stage; closed_ref comes from the read-back; GRASP commands
     closed_ref + 10 deg; GRASP deltas are vs the OPEN row; EE goal
     calls are hover/descend/hover/home with the home yaw
  5. Sequence success path: reached notes, no WARNs
  6. Clamp-stall harness: close stalls -> closed_ref = read-back
     (cmd + 0.2), GRASP = that + 10 deg, run still completes
  7. CSV writer: header + row count + empty cells for None
  8. py_compile + py310-grammar (ast feature_version=(3, 10)) on
     grasp_demo.py, this file, the launch file and both edited
     setup.py files — MANDATORY: the Orin runs python3.10
  9. Topic-rename check: cmd_vel_serial.cpp publishes
     /gripper_pressure_reading (the old "/gripper" publisher is gone)
     and grasp_demo.py subscribes the new name

Exit codes: 0 all pass, 1 a check failed.
"""

import ast
import csv as csv_mod
import math
import os
import py_compile
import sys
import tempfile
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent          # .../sentinel_arm/test
PKG = HERE.parent                               # .../sentinel_arm
SRC = PKG.parent                                # .../ros2_ws/src
ROOT = SRC.parents[1]                           # worktree root
# The sentinel_arm PACKAGE is the inner dir (the outer one is a
# namespace package without __init__.py), so its parent goes on sys.path.
sys.path.insert(0, str(PKG))

import sentinel_arm.grasp_demo as gd  # noqa: E402  (core: no ROS)

GRID_JSON = ROOT / 'generated' / 'grid_5x4_tag36h11_ids18-37.json'
GRASP_DEMO = PKG / 'sentinel_arm' / 'grasp_demo.py'
LAUNCH = (ROOT / 'ros2_ws' / 'src' / 'sentinel_bringup' / 'launch' /
          'grasp_demo.launch.py')
ARM_SETUP = PKG / 'setup.py'
BRINGUP_SETUP = ROOT / 'ros2_ws' / 'src' / 'sentinel_bringup' / 'setup.py'
CMD_VEL_CPP = (ROOT / 'ros2_ws' / 'src' / 'sentinel_drive_cpp' / 'src' /
               'cmd_vel_serial.cpp')

PARAMS = {
    'clamp_close_rad': 0.0,
    'clamp_open_rad': 3.14,
    'clamp_tol': 0.05,
    'grasp_open_deg': 10.0,
    'hover_z': 30.0,
    'descend_z': 10.0,
    'reach_tol': 25.0,
    'reach_timeout': 10.0,
    'clamp_timeout': 5.0,
    'home_xyz': [3.28, -47.44, 69.38, 3.134],
    'min_tags': 4,
}


class FakeLogger:
    def __init__(self):
        self.lines = []

    def info(self, msg):
        self.lines.append(('info', msg))

    def warn(self, msg):
        self.lines.append(('warn', msg))


class FakeEnv:
    """Simulated arm: every wait can be told to fail; the clamp lands at
    the commanded angle (or short of it by +0.2 rad when fail_clamp)."""

    def __init__(self, fail_reach=False, fail_clamp=False):
        self.fail_reach = fail_reach
        self.fail_clamp = fail_clamp
        self.joint_calls = []
        self.ee_calls = []
        self.clamp = None

    def publish_joint_clamp(self, rad):
        self.joint_calls.append(rad)
        self.clamp = float(rad) + (0.2 if self.fail_clamp else 0.0)

    def publish_ee_goal(self, xyz, yaw_rad=None):
        self.ee_calls.append((tuple(np.asarray(xyz, dtype=float)),
                              yaw_rad))

    def wait_reach(self, xyz, timeout):
        if self.fail_reach:
            return False, 99.9, 'err 99.9 mm'
        return True, 1.0, 'reached, err 1.0 mm'

    def wait_clamp(self, rad, timeout):
        if self.fail_clamp:
            return False, self.clamp, \
                f'clamp at {self.clamp:.3f} rad, err 0.200 rad'
        return True, self.clamp, f'reached ({self.clamp:.3f} rad)'

    def read_pressure(self):
        return 123, 456, ''

    def read_clamp(self):
        return self.clamp, ''


def _handeye():
    """A tilted hand-eye pair: grid = R @ arm + t."""
    rx = np.array([[1.0, 0.0, 0.0],
                   [0.0, math.cos(0.5), -math.sin(0.5)],
                   [0.0, math.sin(0.5), math.cos(0.5)]])
    ry = np.array([[math.cos(-0.3), 0.0, math.sin(-0.3)],
                   [0.0, 1.0, 0.0],
                   [-math.sin(-0.3), 0.0, math.cos(-0.3)]])
    r = rx @ ry
    t = np.array([100.0, -60.0, 210.0])
    return r, t


def test_target_square():
    data = gd.load_grid_data(str(GRID_JSON))
    squares = data['squares']
    assert len(squares) == 12, 'grid JSON must have 12 squares'
    ys = [s['center']['y_in'] for s in squares]
    assert ys == sorted(ys), 'squares must be row-major (y increasing)'
    xs = [s['center']['x_in'] for s in squares]
    assert xs[:3] == sorted(xs[:3]), '3 squares per row'
    assert len(set(ys)) == 4, '4 rows of squares'
    x, y = gd.target_square_center(data, 2, 1)
    assert abs(x - 3.7333) < 1e-9 and abs(y - 5.6) < 1e-9, \
        f'row 2 col 1 must be (3.7333, 5.6), got ({x}, {y})'
    assert gd.target_square_center(data, 0, 0) == (1.8667, 1.8667)
    assert gd.target_square_center(data, 3, 2) == (5.6, 7.4667)
    # the documented indexing assumption: 3rd row (1-indexed), between
    # 2nd and 3rd columns -> row index 2, col index 1 -> squares[7]
    assert squares[2 * 3 + 1]['center'] == {'x_in': 3.7333, 'y_in': 5.6}
    for bad in ((4, 0), (0, 3), (-1, 0)):
        try:
            gd.target_square_center(data, *bad)
            raise AssertionError(f'{bad} must be out of range')
        except ValueError:
            pass
    print('PASS target square indexing')


def test_handeye_roundtrip():
    r, t = _handeye()
    grid_mm = np.array([3.7333 * 25.4, 5.6 * 25.4, 0.0])  # the demo square
    arm = gd.grid_to_arm(grid_mm, r, t)
    back = gd.arm_to_grid(arm, r, t)
    assert np.allclose(back, grid_mm, atol=1e-9), 'round-trip must be exact'
    # z-lift: hovering changes ONLY the grid-normal component by hover_z
    n = gd.lift_direction(r)[0]
    assert abs(np.linalg.norm(n) - 1.0) < 1e-9, 'normal must be unit'
    hover_grid = gd.arm_to_grid(arm + 30.0 * n, r, t)
    delta = hover_grid - grid_mm
    assert np.allclose(delta, [0.0, 0.0, 30.0], atol=1e-6), \
        f'lift must be pure grid +z: {delta}'
    print('PASS handeye round-trip + z-lift')


def test_lift_direction_flip():
    r, _ = _handeye()
    # r2 is r with its grid z-axis mirrored (still a proper rotation);
    # exactly one of r / r2 has a through-board grid +z and must flip.
    r2 = np.diag([1.0, -1.0, -1.0]) @ r
    seen_flip = False
    seen_keep = False
    for rr in (r, r2):
        n0 = rr.T @ np.array([0.0, 0.0, 1.0])
        n0 = n0 / np.linalg.norm(n0)
        n, flipped = gd.lift_direction(rr)
        assert abs(np.linalg.norm(n) - 1.0) < 1e-9
        if n0[2] < 0.0:
            assert flipped, 'through-board normal must flip'
            assert np.allclose(n, -n0, atol=1e-9)
            assert n[2] > 0.0
            seen_flip = True
        else:
            assert not flipped, 'away normal must not flip'
            assert np.allclose(n, n0, atol=1e-9)
            seen_keep = True
    assert seen_flip and seen_keep, 'both flip branches must be hit'
    print('PASS lift-direction sign/flip')


def test_deg_to_rad():
    assert gd.deg_to_rad(10.0) == 0.17453292519943295
    assert gd.deg_to_rad(0.0) == 0.0
    assert gd.deg_to_rad(180.0) == math.pi
    print('PASS clamp deg -> rad')


def test_sequence_all_timeouts():
    # EVERY action fails -> the run must still visit all 7 stages in
    # order, WARN per failed stage, and complete (never abort).
    env = FakeEnv(fail_reach=True, fail_clamp=False)
    logger = FakeLogger()
    target = np.array([100.0, -40.0, 150.0])
    normal = np.array([0.0, 0.3, 0.9539392014])
    normal = normal / np.linalg.norm(normal)
    rows = gd.run_sequence(env, target, normal, PARAMS, logger)
    assert [r['stage'] for r in rows] == list(gd.STAGES), \
        f'all 7 stages in order, got {[r["stage"] for r in rows]}'
    for stage in ('HOVER', 'DESCEND', 'LIFT', 'HOME'):
        assert any(lvl == 'warn' and stage in msg
                   for lvl, msg in logger.lines), f'{stage} must WARN'
        row = rows[gd.STAGES.index(stage)]
        assert 'err' in row['note'], f'{stage} row must carry the err'
    # every action fired: close, open, grasp + hover, descend, hover, home
    assert env.joint_calls == [0.0, 3.14,
                               0.0 + gd.deg_to_rad(10.0)]
    assert len(env.ee_calls) == 4
    assert np.allclose(env.ee_calls[0][0],
                       target + 30.0 * normal, atol=1e-6)
    assert np.allclose(env.ee_calls[1][0],
                       target + 10.0 * normal, atol=1e-6)
    assert np.allclose(env.ee_calls[2][0],
                       target + 30.0 * normal, atol=1e-6)
    assert env.ee_calls[3][0] == (3.28, -47.44, 69.38), 'home xyz'
    assert env.ee_calls[3][1] == 3.134, 'home yaw'
    # GRASP deltas vs OPEN (pressure constant in the fake -> 0 deltas)
    grasp_row = rows[gd.STAGES.index('GRASP')]
    open_row = rows[gd.STAGES.index('OPEN')]
    assert grasp_row['delta_clamp_deg'] is not None
    assert abs(grasp_row['delta_clamp_deg'] -
               (math.degrees(gd.deg_to_rad(10.0)) -
                math.degrees(3.14))) < 0.01
    assert grasp_row['delta_hinge'] == 0
    assert grasp_row['delta_anvil'] == 0
    assert open_row['delta_clamp_deg'] is None, 'only GRASP carries deltas'
    print('PASS sequence all-timeouts harness (run completes)')


def test_sequence_success():
    env = FakeEnv(fail_reach=False, fail_clamp=False)
    logger = FakeLogger()
    rows = gd.run_sequence(env, np.array([1.0, 2.0, 3.0]),
                           np.array([0.0, 0.0, 1.0]), PARAMS, logger)
    assert [r['stage'] for r in rows] == list(gd.STAGES)
    assert not any(lvl == 'warn' for lvl, _ in logger.lines), \
        'no WARNs on the success path'
    for stage in ('HOVER', 'DESCEND', 'LIFT', 'HOME'):
        row = rows[gd.STAGES.index(stage)]
        assert 'reached' in row['note'], f'{stage} must report reached'
    print('PASS sequence success harness')


def test_clamp_stall_continues():
    # close/open/grasp stalls: closed_ref = read-back (cmd + 0.2) and
    # GRASP targets that read-back + 10 deg; the run still completes.
    env = FakeEnv(fail_reach=True, fail_clamp=True)
    logger = FakeLogger()
    rows = gd.run_sequence(env, np.array([0.0, 0.0, 100.0]),
                           np.array([0.0, 0.0, 1.0]), PARAMS, logger)
    assert [r['stage'] for r in rows] == list(gd.STAGES)
    for stage in ('CLOSED_REF', 'OPEN', 'GRASP'):
        assert any(lvl == 'warn' and stage in msg
                   for lvl, msg in logger.lines), \
            f'{stage} stall must WARN'
    closed_ref = 0.0 + 0.2  # FakeEnv lands short by +0.2
    grasp = env.joint_calls[2]
    assert abs(grasp - (closed_ref + gd.deg_to_rad(10.0))) < 1e-9, \
        f'grasp = closed_ref + 10 deg, got {grasp}'
    print('PASS clamp-stall harness (closed ref from read-back)')


def test_csv_writer():
    rows = [gd.new_row('OPEN', 3.14, 100, 200, 'reached'),
            gd.new_row('GRASP', 0.1745, 150, 260, 'reached')]
    gd.attach_deltas(rows[1], rows[0])
    with tempfile.TemporaryDirectory() as tmp:
        path = gd.write_log_csv(Path(tmp) / 'log.csv', rows)
        with open(path, newline='') as f:
            read = list(csv_mod.reader(f))
        assert read[0] == list(gd.CSV_FIELDS), 'header row'
        assert len(read) == 3, 'header + 2 rows'
        assert read[2][0] == 'GRASP'
        assert read[1][2] == '179.909', '3.14 rad -> 179.909 deg'
        assert read[2][5] != '', 'GRASP delta_clamp_deg filled'
        assert read[1][5] == '', 'OPEN row has no deltas'
        assert read[1][3] == '100' and read[1][4] == '200', 'pressure'
    print('PASS csv writer')


def test_py310_grammar():
    for path in (GRASP_DEMO, LAUNCH, ARM_SETUP, BRINGUP_SETUP,
                 Path(__file__).resolve()):
        source = path.read_text()
        ast.parse(source, filename=str(path), feature_version=(3, 10))
        py_compile.compile(str(path), doraise=True)
        print(f'PASS py310-grammar + py_compile: {path.name}')


def test_gripper_topic_rename():
    cpp = CMD_VEL_CPP.read_text()
    assert '/gripper_pressure_reading' in cpp, \
        'cmd_vel_serial.cpp must publish the renamed topic'
    assert '"/gripper"' not in cpp, 'old /gripper publisher must be gone'
    src = GRASP_DEMO.read_text()
    assert "'gripper_pressure_reading'" in src, \
        'grasp_demo.py must subscribe the renamed topic'
    print('PASS gripper topic rename')


def main():
    print('T007 selftest (no ROS, no camera, no detector)')
    test_target_square()
    test_handeye_roundtrip()
    test_lift_direction_flip()
    test_deg_to_rad()
    test_sequence_all_timeouts()
    test_sequence_success()
    test_clamp_stall_continues()
    test_csv_writer()
    test_py310_grammar()
    test_gripper_topic_rename()
    print('RESULT: PASS — all T007 selftests green')
    return 0


if __name__ == '__main__':
    sys.exit(main())
