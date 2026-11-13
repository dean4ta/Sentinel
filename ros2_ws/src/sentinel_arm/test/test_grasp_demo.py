#!/usr/bin/env python3
"""T007/T008 selftest — Mac-side, no ROS, no camera, no detector.

Exercises the importable core of grasp_demo.py (the ROS node class is
guarded behind the rclpy import, so this runs in plain Python):

  1. Target-square indexing against the real grid JSON: 12 squares,
     row-major; (row 2, col 1) = squares[7] = (3.7333, 5.6) in = the
     "3rd row, between 2nd and 3rd columns" square; out-of-range raises
  2. Hand-eye transform round-trip (grid -> arm -> grid exact) and the
     z-lift math: target + hover_z along the signed normal changes only
     the grid-normal component by exactly hover_z; lift_direction
     normalizes, flips a through-board R and keeps an away-from-board R
  3. clamp deg -> rad (10 deg = 0.17453292519943295 rad); SettleTracker
     (the vacuous-reach fix): moving samples INSIDE tol never settle
     early, static in-tol settles after N+1 feeds, out-of-tol never,
     reset() (stale-pose gap) restarts the consecutive run
  4. Sequence harness, all actions failing: all 7 stages still run in
     order and the run COMPLETES; each failed action logs a WARN naming
     the stage; closed_ref is logged from the read-back; GRASP commands
     the direct grasp_clamp_rad (2.686); GRASP deltas are vs the OPEN
     row; EE goal calls are hover(60 mm)/descend/hover(60 mm)/home with
     the home yaw; DESCEND and LIFT also run the settle check
     (wait_settled), whose failure WARNs and continues
  5. Sequence success path: reached notes, no WARNs
  6. Clamp-stall harness: close/open stall -> read-back logged short
     of cmd, GRASP still commands grasp_clamp_rad directly, run still
     completes
  7. CSV writer: header + row count + empty cells for None
  8. T008 live localization (pure math, no camera): synthetic PnP
     round-trip (projectPoints from a known grid->cam pose, solvePnP
     recovers it within tolerance, behind-camera/degenerate -> None);
     T_cam_in_arm composition (cam->arm->grid chain round-trips and
     equals inv(handeye) @ inv(pnp)); moved-grid simulation (two
     different cam_T_grid -> two DIFFERENT target_arm, the second
     equals the chain result, the first equals the static handeye
     target); trigger freshness gate (fresh live -> live, stale/none ->
     static fallback + WARN reason); live normal direction + sign
     logic for both live paths
  9. py_compile + py310-grammar (ast feature_version=(3, 10)) on
     grasp_demo.py, arm_bridge.py, this file, the launch file and both
     edited setup.py files — MANDATORY: the Orin runs python3.10
 10. Topic-rename check: cmd_vel_serial.cpp publishes
     /gripper_pressure_reading (the old "/gripper" publisher is gone)
     and grasp_demo.py subscribes the new name
 11. hover_z default 60.0 in grasp_demo.py (the T008 change; the old
     30.0 must be gone)

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

import cv2
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
ARM_BRIDGE = PKG / 'sentinel_arm' / 'arm_bridge.py'
LAUNCH = (ROOT / 'ros2_ws' / 'src' / 'sentinel_bringup' / 'launch' /
          'grasp_demo.launch.py')
ARM_SETUP = PKG / 'setup.py'
BRINGUP_SETUP = ROOT / 'ros2_ws' / 'src' / 'sentinel_bringup' / 'setup.py'
CMD_VEL_CPP = (ROOT / 'ros2_ws' / 'src' / 'sentinel_drive_cpp' / 'src' /
               'cmd_vel_serial.cpp')

PARAMS = {
    'clamp_close_rad': 3.140,
    'clamp_open_rad': 1.859,
    'clamp_tol': 0.05,
    'grasp_clamp_rad': 2.686,
    'hover_z': 60.0,   # T008: 30 -> 60 mm
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
        self.settle_calls = []
        self.clamp = None

    def publish_joint_clamp(self, rad):
        self.joint_calls.append(rad)
        self.clamp = float(rad) + (0.2 if self.fail_clamp else 0.0)

    def publish_ee_goal(self, xyz, yaw_rad=None):
        self.ee_calls.append(
            (tuple(float(v) for v in np.asarray(xyz, dtype=float)),
             yaw_rad))

    def wait_reach(self, xyz, timeout):
        if self.fail_reach:
            return False, 99.9, 'err 99.9 mm'
        return True, 1.0, 'reached, err 1.0 mm'

    def wait_settled(self, xyz, timeout):
        self.settle_calls.append(
            (tuple(float(v) for v in np.asarray(xyz, dtype=float)),
             timeout))
        if self.fail_reach:
            return False, 99.9, 'still moving (12.3 mm/sample)'
        return True, 1.0, 'settled, err 1.0 mm'

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
    hover_grid = gd.arm_to_grid(arm + PARAMS['hover_z'] * n, r, t)
    delta = hover_grid - grid_mm
    assert np.allclose(delta, [0.0, 0.0, PARAMS['hover_z']], atol=1e-6), \
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


def test_settle_tracker():
    # The vacuous-reach fix: hover/descend targets are ~20 mm apart
    # (inside the 25 mm tol), so a plain err <= tol check passes while
    # the arm is still moving. SettleTracker must NOT pass early on
    # moving samples inside the tolerance, and must pass on static
    # in-tolerance samples after SETTLE_N_SAMPLES + 1 fresh feeds.
    cmd = np.array([0.0, 0.0, 100.0])
    tr = gd.SettleTracker(tol=25.0)
    # moving samples at ~20 mm err (INSIDE tol): never settles
    p = np.array([20.0, 0.0, 100.0])
    for step in range(12):
        settled, err, note = tr.feed(
            p + step * np.array([-2.0, 0.0, 0.0]), cmd)
        assert not settled, 'moving in-tol samples must not settle'
        assert err <= 25.0
    assert 'still moving' in note
    # static in-tolerance samples: settles exactly on the N+1-th feed
    tr.reset()
    settled = False
    for i in range(gd.SETTLE_N_SAMPLES + 1):
        settled, err, note = tr.feed(np.array([1.0, 0.0, 100.0]), cmd)
        if i < gd.SETTLE_N_SAMPLES:
            assert not settled, f'settled too early (feed {i})'
    assert settled, 'static in-tol samples must settle'
    assert 'settled' in note
    # outside tolerance: never settles, however many samples
    tr.reset()
    for _ in range(20):
        settled, _, _ = tr.feed(np.array([99.0, 0.0, 100.0]), cmd)
        assert not settled, 'out-of-tol samples must not settle'
    # a reset (stale-pose gap) breaks the consecutive run
    tr.reset()
    for _ in range(3):
        settled, _, _ = tr.feed(np.array([1.0, 0.0, 100.0]), cmd)
        assert not settled, 'consecutive run must restart after reset'
    print('PASS settle tracker (moving in-tol does not pass early)')


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
    # every action fired: close, open, grasp (direct grasp_clamp_rad)
    assert env.joint_calls == [3.140, 1.859, 2.686]
    assert len(env.ee_calls) == 4
    assert np.allclose(env.ee_calls[0][0],
                       target + PARAMS['hover_z'] * normal, atol=1e-6)
    assert np.allclose(env.ee_calls[1][0],
                       target + 10.0 * normal, atol=1e-6)
    assert np.allclose(env.ee_calls[2][0],
                       target + PARAMS['hover_z'] * normal, atol=1e-6)
    assert env.ee_calls[3][0] == (3.28, -47.44, 69.38), 'home xyz'
    assert env.ee_calls[3][1] == 3.134, 'home yaw'
    # settle check runs after DESCEND and LIFT only (the vacuous-reach
    # fix): both fail in this harness and must WARN + continue
    assert [c[0] for c in env.settle_calls] == \
        [env.ee_calls[1][0], env.ee_calls[2][0]], \
        'settle check must follow DESCEND and LIFT (only)'
    assert all(c[1] == PARAMS['reach_timeout']
               for c in env.settle_calls), 'settle timeout = reach_timeout'
    for stage in ('DESCEND', 'LIFT'):
        assert any(lvl == 'warn' and f'{stage}: not settled' in msg
                   for lvl, msg in logger.lines), \
            f'{stage} settle failure must WARN'
        row = rows[gd.STAGES.index(stage)]
        assert 'still moving' in row['note'], \
            f'{stage} row must carry the settle note'
    # GRASP deltas vs OPEN (pressure constant in the fake -> 0 deltas)
    grasp_row = rows[gd.STAGES.index('GRASP')]
    open_row = rows[gd.STAGES.index('OPEN')]
    assert grasp_row['delta_clamp_deg'] is not None
    assert abs(grasp_row['delta_clamp_deg'] -
               (math.degrees(2.686) - math.degrees(1.859))) < 0.01
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
    assert [c[0] for c in env.settle_calls] == \
        [env.ee_calls[1][0], env.ee_calls[2][0]], \
        'settle check must follow DESCEND and LIFT (only)'
    for stage in ('DESCEND', 'LIFT'):
        row = rows[gd.STAGES.index(stage)]
        assert 'settled' in row['note'], f'{stage} must report settled'
    print('PASS sequence success harness')


def test_clamp_stall_continues():
    # close/open/grasp stalls: read-back angles logged short of cmd;
    # GRASP still commands the direct grasp_clamp_rad (2.686) — no
    # closed_ref + 10 deg computation anymore; the run still completes.
    env = FakeEnv(fail_reach=True, fail_clamp=True)
    logger = FakeLogger()
    rows = gd.run_sequence(env, np.array([0.0, 0.0, 100.0]),
                           np.array([0.0, 0.0, 1.0]), PARAMS, logger)
    assert [r['stage'] for r in rows] == list(gd.STAGES)
    for stage in ('CLOSED_REF', 'OPEN', 'GRASP'):
        assert any(lvl == 'warn' and stage in msg
                   for lvl, msg in logger.lines), \
            f'{stage} stall must WARN'
    assert env.joint_calls[2] == 2.686, \
        f'grasp must be the direct grasp_clamp_rad, ' \
        f'got {env.joint_calls[2]}'
    print('PASS clamp-stall harness (grasp = direct grasp_clamp_rad)')


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


def _synthetic_pnp_case():
    """A known grid->cam pose: real grid JSON tag corners (black-border
    at 0.8 x side, inches) projected through a synthetic K + dist.
    Returns (obj_in (M,2), img_px (M,2), k, dist, t_grid_in_cam)."""
    k = np.array([[600.0, 0.0, 320.0],
                  [0.0, 600.0, 240.0],
                  [0.0, 0.0, 1.0]])
    dist = np.zeros(5)
    # board roughly 520 mm in front of the camera, tilted ~23 deg x
    # and ~15 deg y — a plausible over-the-table view
    ry = np.array([[math.cos(0.15), 0.0, math.sin(0.15)],
                   [0.0, 1.0, 0.0],
                   [-math.sin(0.15), 0.0, math.cos(0.15)]])
    rx = np.array([[1.0, 0.0, 0.0],
                   [0.0, math.cos(0.4), -math.sin(0.4)],
                   [0.0, math.sin(0.4), math.cos(0.4)]])
    r = ry @ rx
    t = np.array([-40.0, 30.0, 520.0])
    t_grid_in_cam = gd.tf4(r, t)
    grid = gd.load_grid(str(GRID_JSON))
    obj_in, obj_mm = [], []
    for tid in sorted(grid)[:6]:  # 6 tags x 4 corners = 24 points
        cx, cy, side = grid[tid]
        for (x, y) in gd.tag_object_corners(cx, cy, side):
            obj_in.append((x, y))
            obj_mm.append((x * 25.4, y * 25.4, 0.0))
    obj_mm = np.array(obj_mm, dtype=float)
    img, _ = cv2.projectPoints(obj_mm, cv2.Rodrigues(r)[0], t, k, dist)
    return obj_in, img.reshape(-1, 2), k, dist, t_grid_in_cam


def test_pnp_roundtrip():
    # synthetic PnP round-trip: projectPoints from a known grid->cam
    # pose, then solvePnP must recover that pose within tolerance.
    obj_in, img, k, dist, t_true = _synthetic_pnp_case()
    t_est = gd.pnp_grid_pose(obj_in, img, k, dist)
    assert t_est is not None, 'synthetic PnP must succeed'
    err_t = float(np.linalg.norm(t_est[:3, 3] - t_true[:3, 3]))
    cosang = float(np.clip(
        (np.trace(t_est[:3, :3].T @ t_true[:3, :3]) - 1.0) / 2.0,
        -1.0, 1.0))
    err_r = math.degrees(math.acos(cosang))
    assert err_t < 5.0, f'translation error {err_t:.1f} mm too large'
    assert err_r < 2.0, f'rotation error {err_r:.2f} deg too large'
    # degenerate inputs must return None, not raise
    assert gd.pnp_grid_pose(obj_in[:3], img[:3], k, dist) is None, \
        '< 4 points must fail cleanly'
    assert gd.pnp_grid_pose(obj_in, img, np.zeros((3, 3)), dist) is None, \
        'a zero K must fail cleanly'
    print('PASS PnP round-trip (projectPoints -> solvePnP) + failure '
          'branches')


def test_arm_cam_chain():
    # T_cam_in_arm = inv(handeye 4x4) @ inv(T_grid_in_cam); the
    # cam->arm->grid chain must round-trip exactly against the handeye
    # equation.
    r_h, t_h = _handeye()
    obj_in, img, k, dist, t_grid_in_cam = _synthetic_pnp_case()
    t_cam_in_arm = gd.arm_cam_tf(r_h, t_h, t_grid_in_cam)
    assert np.allclose(
        t_cam_in_arm,
        gd.arm_tf_from_handeye(r_h, t_h) @ gd.inv_tf(t_grid_in_cam),
        atol=1e-12), 'must equal inv(handeye) @ inv(pnp)'
    p_cam = np.array([123.0, -45.0, 500.0])
    p_arm = gd.apply_tf(t_cam_in_arm, p_cam)
    p_grid_via_cam = gd.apply_tf(gd.inv_tf(t_grid_in_cam), p_cam)
    assert np.allclose(gd.arm_to_grid(p_arm, r_h, t_h),
                       p_grid_via_cam, atol=1e-9), \
        'cam -> arm -> grid must round-trip'
    print('PASS T_cam_in_arm composition (cam -> arm -> grid chain)')


def test_moved_grid():
    # two different cam_T_grid -> two DIFFERENT target_arm; the first
    # equals the static handeye target (board in the calibration
    # position at capture), the second equals the chain result.
    r_h, t_h = _handeye()
    obj_in, img, k, dist, t_a = _synthetic_pnp_case()
    yaw = 0.35
    m = gd.tf4(np.array([[math.cos(yaw), -math.sin(yaw), 0.0],
                         [math.sin(yaw), math.cos(yaw), 0.0],
                         [0.0, 0.0, 1.0]]),
               np.array([120.0, -40.0, 0.0]))
    t_b = t_a @ m  # the grid moved by m between detections
    t_cam_in_arm = gd.arm_cam_tf(r_h, t_h, t_a)  # captured at pose A
    g = np.array([3.7333, 5.6, 0.0]) * 25.4        # the demo square
    live_a = gd.live_target_from(t_cam_in_arm, t_a, g)
    live_b = gd.live_target_from(t_cam_in_arm, t_b, g)
    static = gd.grid_to_arm(g, r_h, t_h)
    assert np.allclose(live_a, static, atol=1e-6), \
        'at capture the live target must equal the static handeye target'
    assert np.linalg.norm(live_b - live_a) > 50.0, \
        'a moved grid must move the target'
    assert np.allclose(live_b,
                       gd.grid_to_arm(gd.apply_tf(m, g), r_h, t_h),
                       atol=1e-6), \
        'the moved-grid target must equal the chain result'
    print('PASS moved-grid live target (A = static, B = chain result)')


def test_live_normal():
    # the live normal must equal the static one (the camera is fixed to
    # the arm) and the sign logic must flip a through-board normal on
    # both paths.
    r_h, t_h = _handeye()
    obj_in, img, k, dist, t_a = _synthetic_pnp_case()
    t_cam_in_arm = gd.arm_cam_tf(r_h, t_h, t_a)
    n = gd.live_normal_from(t_cam_in_arm, t_a)
    assert abs(np.linalg.norm(n) - 1.0) < 1e-9, 'live normal must be unit'
    assert np.allclose(n, gd.lift_direction(r_h)[0], atol=1e-9), \
        'live normal == static normal (camera fixed to the arm)'
    n_keep, flipped_keep = gd.sign_lift_normal(n)
    assert not flipped_keep and np.allclose(n_keep, n, atol=1e-9)
    # through-board handeye: the live normal must flip too
    r2 = np.diag([1.0, -1.0, -1.0]) @ r_h
    t2_cam_in_arm = gd.arm_cam_tf(r2, t_h, t_a)
    n2, flipped = gd.sign_lift_normal(
        gd.live_normal_from(t2_cam_in_arm, t_a))
    assert flipped, 'through-board live normal must flip'
    assert n2[2] > 0.0, 'the flipped live normal must point away'
    print('PASS live normal direction + sign logic (both live paths)')


def test_static_fallback():
    # the T008 trigger freshness gate: freshest live target when
    # younger than DETECT_MAX_AGE_S; otherwise static fallback + WARN
    # reason.
    static = np.array([100.0, -40.0, 150.0])
    s_normal = np.array([0.0, 0.0, 1.0])
    live = np.array([220.0, -80.0, 130.0])
    l_normal = np.array([0.0, 0.3, 0.9539392014])
    l_normal = l_normal / np.linalg.norm(l_normal)
    now = 1000.0
    t, n, used, reason = gd.pick_target(live, l_normal, now - 1.0, now,
                                        static, s_normal)
    assert used and not reason, 'a fresh live target must be used'
    assert np.allclose(t, live) and np.allclose(n, l_normal)
    t, n, used, reason = gd.pick_target(live, l_normal, now - 6.0, now,
                                        static, s_normal)
    assert not used and 'stale' in reason, 'a stale live target must WARN'
    assert np.allclose(t, static) and np.allclose(n, s_normal)
    t, n, used, _ = gd.pick_target(
        live, l_normal, now - gd.DETECT_MAX_AGE_S, now, static, s_normal)
    assert used, 'exactly DETECT_MAX_AGE_S old is still fresh (<=)'
    t, n, used, reason = gd.pick_target(None, None, None, now,
                                        static, s_normal)
    assert not used and 'no live target' in reason, \
        'no live target must fall back with a reason'
    assert np.allclose(t, static)
    print('PASS trigger freshness gate (stale -> static fallback + WARN)')


def test_hover_default_60():
    src = GRASP_DEMO.read_text()
    assert "declare_parameter('hover_z', 60.0)" in src, \
        'hover_z default must be 60.0 (T008)'
    assert "'hover_z', 30.0" not in src, 'the old 30.0 default must be gone'
    print('PASS hover_z default 60.0')


def test_py310_grammar():
    for path in (GRASP_DEMO, ARM_BRIDGE, LAUNCH, ARM_SETUP, BRINGUP_SETUP,
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
    print('T007/T008 selftest (no ROS, no camera, no detector)')
    test_target_square()
    test_handeye_roundtrip()
    test_lift_direction_flip()
    test_deg_to_rad()
    test_settle_tracker()
    test_sequence_all_timeouts()
    test_sequence_success()
    test_clamp_stall_continues()
    test_csv_writer()
    test_pnp_roundtrip()
    test_arm_cam_chain()
    test_moved_grid()
    test_live_normal()
    test_static_fallback()
    test_hover_default_60()
    test_py310_grammar()
    test_gripper_topic_rename()
    print('RESULT: PASS — all T007/T008 selftests green')
    return 0


if __name__ == '__main__':
    sys.exit(main())
