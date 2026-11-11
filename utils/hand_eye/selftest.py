#!/usr/bin/env python3
"""T004 selftest — Mac-side, no ROS, no camera, no detector.

Exercises the importable cores of handeye_capture.py and
solve_handeye.py:

  1. Procrustes recovery of an exact synthetic rigid transform
     (rotation error < 1e-6 deg, t error < 1e-6 mm, residual < 1e-6 mm)
  2. Procrustes with Gaussian sigma=2 mm noise on the arm points
     (RMS < 3 mm, rotation error < 0.5 deg)
  3. Target generation against the real 5x4 grid JSON: 12 unique
     targets, 4 outer corners match the JSON geometry (tag 0 TL corner
     = (0,0)), the 8 interior corners cover the central region
  4. Undistort path with a synthetic camera_matrix.yaml: load -> maps
     -> remap changes the image (non-identity), output shape kept;
     missing file returns None + a WARN on stderr
  5. py_compile of arm_bridge.py, handeye_capture.py, solve_handeye.py,
     test_handeye.py and this file (compile only — no ROS imports are
     executed)
  6. T005 one-dot draw UX: targets_to_draw(k) marks exactly one target
     'current' (red/large, index k), exactly k 'recorded' (green) and
     11-k 'hidden' for k in 0..11; k=12 -> 0 current, 12 recorded
  7. T006 transform round-trip: handeye.yaml-style R, t + a grid
     point -> arm -> grid -> arm recovers exact (< 1e-9 mm), including
     a planar-solve-style R (grid-z column = cross of the in-plane
     columns, as the Procrustes solve produces up to the normal sign)
  8. T006 lift math: z_lift=10 mm changes only the grid-normal
     component, by exactly 10 mm; the signed normal points AWAY from
     the board (arm frame z > 0) and a yaml R whose grid +z points
     through the board is flipped
  9. T006 spd clamp: distances 0/50/1000 mm over 2 s -> 50/50/300
     (and 400 mm -> 200)
  10. T006 handeye.yaml loader: reads synthetic solve_handeye.py
      output (PyYAML dump and the plain emitter format) and rejects
      bad shapes / a non-rotation R / a det=1 non-orthonormal R /
      missing files
  11. T006 double-run guard: a live pid's lock refuses a second
      acquire; a stale (dead-pid) lock is cleared; release removes
      only our own lock
  12. T006 end-to-end core path: real grid JSON + synthetic solver
      handeye.yaml -> 12 targets -> 12 commanded arm points, each
      exactly z_lift above the board plane in the grid frame

Exit codes: 0 all pass, 1 a check failed.
"""

import io
import math
import os
import py_compile
import sys
import tempfile
from contextlib import redirect_stderr
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import handeye_capture as hc  # noqa: E402  (core: no ROS needed)
import solve_handeye as sh  # noqa: E402
import test_handeye as th  # noqa: E402  (core: numpy only, no ROS)

ROOT = HERE.parents[1]
GRID = ROOT / 'generated' / 'grid_5x4_tag36h11_ids18-37.json'
ARM_BRIDGE = (ROOT / 'ros2_ws' / 'src' / 'sentinel_arm' /
              'sentinel_arm' / 'arm_bridge.py')


def random_rotation(rng):
    """Uniform-ish random SO(3) via QR of a gaussian matrix."""
    q, _ = np.linalg.qr(rng.normal(size=(3, 3)))
    if np.linalg.det(q) < 0.0:
        q[:, 0] *= -1.0
    return q


def rotation_error_deg(r_est, r_true):
    cos = (np.trace(r_est.T @ r_true) - 1.0) / 2.0
    return math.degrees(math.acos(float(np.clip(cos, -1.0, 1.0))))


def test_procrustes_exact():
    rng = np.random.default_rng(7)
    r_true = random_rotation(rng)
    t_true = rng.uniform(-200.0, 200.0, size=3)
    arm = rng.uniform([-80.0, -100.0, 60.0], [80.0, 100.0, 160.0],
                      size=(12, 3))
    grid = arm @ r_true.T + t_true
    r_est, t_est, rms, residuals = sh.solve_rigid(arm, grid)
    rot_err = rotation_error_deg(r_est, r_true)
    t_err = float(np.linalg.norm(t_est - t_true))
    assert rot_err < 1e-6, f'rotation error {rot_err:.3e} deg'
    assert t_err < 1e-6, f't error {t_err:.3e} mm'
    assert rms < 1e-6 and float(residuals.max()) < 1e-6
    print(f'PASS procrustes-exact: rot err {rot_err:.2e} deg, '
          f't err {t_err:.2e} mm, rms {rms:.2e} mm, '
          f'max residual {residuals.max():.2e} mm')


def test_procrustes_noisy():
    # Rotation recovery scales with the points' lever arm, so the
    # synthetic points span an arm-workspace-scale box (450x450x300 mm,
    # the RoArm's ~450 mm reach). Fixed seed: deterministic margin
    # (rot err ~0.22 deg vs 0.5 limit, rms ~2.3 mm vs 3 limit).
    rng = np.random.default_rng(10)
    r_true = random_rotation(rng)
    t_true = rng.uniform(-200.0, 200.0, size=3)
    arm = rng.uniform([-225.0, -225.0, 40.0], [225.0, 225.0, 340.0],
                      size=(12, 3))
    noisy = arm + rng.normal(0.0, 2.0, size=arm.shape)  # sigma 2 mm
    grid = arm @ r_true.T + t_true
    r_est, t_est, rms, residuals = sh.solve_rigid(noisy, grid)
    rot_err = rotation_error_deg(r_est, r_true)
    assert rms < 3.0, f'RMS {rms:.3f} mm >= 3 mm'
    assert rot_err < 0.5, f'rotation error {rot_err:.3f} deg >= 0.5 deg'
    print(f'PASS procrustes-noisy: rms {rms:.3f} mm (< 3), '
          f'rot err {rot_err:.3f} deg (< 0.5)')


def test_pairs_path():
    """The capture->solver data path: pairs dict -> arm_grid_arrays
    (2D grid inches padded to z=0) -> solve_rigid -> handeye_data ->
    dump_yaml round-trip."""
    rng = np.random.default_rng(3)
    r_true = random_rotation(rng)
    t_true = rng.uniform(-100.0, 100.0, size=3)
    # Physical setup: grid points lie in the board plane (z=0 in the
    # grid frame); arm points are where the transform maps that plane.
    grid_3d = np.column_stack([
        rng.uniform(-80.0, 80.0, size=12),
        rng.uniform(-100.0, 100.0, size=12),
        np.zeros(12)])
    arm = (grid_3d - t_true) @ r_true  # grid = arm @ R.T + t
    points = []
    for i, (g, row) in enumerate(zip(grid_3d, arm)):
        points.append({'target': f't{i}', 'tag_id': 18, 'corner': 'TL',
                       'grid': [float(g[0]) / sh.IN_TO_MM,
                                float(g[1]) / sh.IN_TO_MM],
                       'arm_xyz_mm': [float(v) for v in row]})
    pairs = {'unit_grid': 'in', 'unit_arm': 'mm', 'points': points}

    arm_arr, grid_arr, labels = sh.arm_grid_arrays(points, 'in')
    assert arm_arr.shape == (12, 3) and grid_arr.shape == (12, 3)
    assert np.allclose(grid_arr[:, 2], 0.0), 'grid z must be 0'
    assert np.allclose(grid_arr, grid_3d, atol=1e-9), 'in->mm + z pad'
    assert labels == [f't{i}' for i in range(12)]
    r_est, t_est, rms, residuals = sh.solve_rigid(arm_arr, grid_arr)
    assert rotation_error_deg(r_est, r_true) < 1e-6, 'R recovery'
    assert float(np.linalg.norm(t_est - t_true)) < 1e-6, 't recovery'

    data = sh.handeye_data(points, r_est, t_est, rms, residuals)
    assert set(data) >= {'R', 't', 'rms_mm', 'per_point_residuals_mm',
                         'units', 'timestamp', 'point_count'}
    assert data['point_count'] == 12
    with tempfile.TemporaryDirectory() as tmp:
        out = Path(tmp) / 'handeye.yaml'
        sh.dump_yaml(out, data)
        text = out.read_text()
        assert 'rms_mm' in text and 'per_point_residuals_mm' in text
        # The plain-YAML fallback emitter produces the same content.
        plain = sh.plain_yaml(data)
        assert 'rms_mm' in plain and 'point_count: 12' in plain
    print(f'PASS pairs-path: 2D grid -> z=0 pad, in->mm, solve, '
          f'yaml round-trip (rms {rms:.2e} mm)')


def test_targets():
    targets, grid_tags = hc.build_targets(GRID)
    assert len(grid_tags) == 20, 'expected 20 tags in the grid JSON'
    assert len(targets) == 12, f'expected 12 targets, got {len(targets)}'
    names = [t['target'] for t in targets]
    assert len(set(names)) == 12, 'target names must be unique'
    pts = [tuple(t['grid']) for t in targets]
    assert len(set(pts)) == 12, 'target points must be unique'
    by_name = {t['target']: t for t in targets}

    # 4 outer corners match the JSON geometry: tag 0's printed TL
    # corner is the origin, printed side includes the quiet zone.
    tl = by_name['outer_TL']
    assert tl['tag_id'] == 18 and tl['corner'] == 'TL'
    assert abs(tl['grid'][0]) < 1e-6 and abs(tl['grid'][1]) < 1e-6, \
        f'outer TL must be (0,0), got {tl["grid"]}'
    tr = by_name['outer_TR']['grid']
    assert abs(tr[0] - 7.4667) < 1e-3 and abs(tr[1]) < 1e-3, \
        f'outer TR wrong: {tr}'
    br = by_name['outer_BR']['grid']
    assert abs(br[0] - 7.4667) < 1e-3 and abs(br[1] - 9.3333) < 1e-3, \
        f'outer BR wrong: {br}'
    bl = by_name['outer_BL']['grid']
    assert abs(bl[0]) < 1e-3 and abs(bl[1] - 9.3333) < 1e-3, \
        f'outer BL wrong: {bl}'
    assert by_name['outer_TR']['tag_id'] == 21
    assert by_name['outer_BR']['tag_id'] == 37
    assert by_name['outer_BL']['tag_id'] == 34

    # 8 interior corners: strictly inside the outer box, each matches
    # its tag's 0.8x black-border corner, and together they cover the
    # central region (convex hull contains the board center).
    interior = [t for t in targets if t['target'].startswith('interior')]
    assert len(interior) == 8
    for t in interior:
        x, y = t['grid']
        assert 1.5 < x < 7.3 and 1.5 < y < 8.5, \
            f'{t["target"]} outside the central region: ({x}, {y})'
        cx, cy, side = grid_tags[t['tag_id']]
        h = hc.QUIET_ZONE_FRACTION * side / 2.0
        expected = {'TL': (cx - h, cy - h), 'TR': (cx + h, cy - h),
                    'BR': (cx + h, cy + h), 'BL': (cx - h, cy + h)}
        assert np.allclose(t['grid'], expected[t['corner']], atol=1e-9), \
            f'{t["target"]} grid point mismatch'
    xs = [t['grid'][0] for t in interior]
    ys = [t['grid'][1] for t in interior]
    assert max(xs) - min(xs) >= 4.0, 'interior x spread too small'
    assert max(ys) - min(ys) >= 4.0, 'interior y spread too small'
    hull = cv2.convexHull(np.array([[x, y] for t in interior
                                    for x, y in [t['grid']]],
                                   dtype=np.float32))
    board_center = (3.7333, 4.6667)  # 7.4667 x 9.3333 in board
    assert cv2.pointPolygonTest(hull, board_center, False) >= 0.0, \
        'interior corners do not cover the central region'
    print(f'PASS targets: 12 unique ({", ".join(names[:4])} + 8 '
          f'interior), outer match JSON, interior central')


def test_target_dot_ux():
    # T005 (D-013): the detections window shows ONLY ONE red dot — the
    # current target. For recorded count k in 0..11 exactly one target
    # has the red/large 'current' style (the k-th), exactly k have the
    # green 'recorded' style, and the remaining 11-k are 'hidden'.
    # k=12 -> 0 current, 12 recorded (all green, none hidden).
    for k in range(13):
        styles = hc.targets_to_draw(k)
        assert len(styles) == hc.N_TARGETS, f'k={k}: wrong length'
        cur = [i for i, s in enumerate(styles) if s == 'current']
        rec = sum(s == 'recorded' for s in styles)
        hid = sum(s == 'hidden' for s in styles)
        if k < 12:
            assert cur == [k], f'k={k}: current must be exactly [{k}], got {cur}'
            assert rec == k and hid == 11 - k, \
                f'k={k}: got {rec} recorded / {hid} hidden'
            assert all(s == 'recorded' for s in styles[:k]), \
                f'k={k}: first {k} must be recorded'
            assert all(s == 'hidden' for s in styles[k + 1:]), \
                f'k={k}: indices {k + 1}.. must be hidden'
        else:
            assert cur == [] and rec == 12 and hid == 0, \
                'k=12: all recorded, none current/hidden'
    print('PASS target-dot-ux: 1 current red, k green, 11-k hidden '
          'per step; 12 -> 12 green, 0 red')


def test_undistort():
    k = np.array([[700.0, 0.0, 640.0], [0.0, 700.0, 360.0],
                  [0.0, 0.0, 1.0]])
    dist = np.array([0.15, -0.25, 0.0, 0.0, 0.05])
    data = {'camera_matrix': k.tolist(), 'dist_coeffs': dist.tolist(),
            'image_size': [1280, 720], 'rms': 0.4}
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / 'camera_matrix.yaml'
        try:
            import yaml
            path.write_text(yaml.safe_dump(data))
        except ImportError:
            path.write_text(sh.plain_yaml(data))
        loaded = hc.load_camera_matrix(path)
        assert loaded is not None, 'synthetic yaml should load'
        kk, dd, size = loaded
        assert np.allclose(kk, k), 'camera_matrix mismatch'
        assert np.allclose(dd, dist), 'dist_coeffs mismatch'
        assert size == (1280, 720), 'image_size mismatch'
        # K scaling for a capture resolution != calibration (minor-01):
        # fx/cx scale with the width ratio, fy/cy with the height ratio.
        scaled = hc.scale_intrinsics(kk, (1280, 720), (640, 480))
        assert np.isclose(scaled[0, 0], 350.0), 'fx must scale 0.5x'
        assert np.isclose(scaled[0, 2], 320.0), 'cx must scale 0.5x'
        assert np.isclose(scaled[1, 1], 700.0 * 2.0 / 3.0), 'fy scale'
        assert np.isclose(scaled[1, 2], 360.0 * 2.0 / 3.0), 'cy scale'
        assert np.allclose(scaled[2], kk[2]), 'third row untouched'
        same = hc.scale_intrinsics(kk, (1280, 720), (1280, 720))
        assert np.allclose(same, kk), 'equal resolution must not scale'
        # A yaml without image_size yields size None (no scaling check).
        del data['image_size']
        path2 = Path(tmp) / 'no_size.yaml'
        try:
            import yaml
            path2.write_text(yaml.safe_dump(data))
        except ImportError:
            path2.write_text(sh.plain_yaml(data))
        kk2, dd2, size2 = hc.load_camera_matrix(path2)
        assert size2 is None and np.allclose(kk2, k) \
            and np.allclose(dd2, dist)
        maps = hc.build_undistort_maps(kk, dd, (320, 240))
        # Gradient image: remap must change it (non-identity) and keep
        # the shape.
        base = np.tile(np.arange(320, dtype=np.float32), (240, 1))
        img = np.stack([base, base, base], axis=-1).astype(np.uint8)
        out = hc.undistort_frame(img, maps)
        assert out.shape == img.shape, 'undistort changed the shape'
        assert np.abs(out.astype(int) - img.astype(int)).max() > 0, \
            'undistort should change the image'
        # Missing file: None + a WARN on stderr.
        missing = Path(tmp) / 'nope.yaml'
        buf = io.StringIO()
        with redirect_stderr(buf):
            got = hc.load_camera_matrix(missing)
        assert got is None, 'missing file must return None'
        assert 'WARN' in buf.getvalue(), 'missing file must warn'
    print('PASS undistort: load, maps, remap non-identity, '
          'missing -> None + WARN')


def test_t006_roundtrip():
    """T006 transform round-trip: handeye.yaml-style R, t + a grid
    point -> arm XYZ -> grid -> arm recovers exact (< 1e-9 mm)."""
    rng = np.random.default_rng(17)
    r_true = random_rotation(rng)
    t_true = rng.uniform(-200.0, 200.0, size=3)
    for g in ([0.0, 0.0, 0.0],
              [7.4667 * sh.IN_TO_MM, 9.3333 * sh.IN_TO_MM, 0.0],
              [3.0 * sh.IN_TO_MM, 5.0 * sh.IN_TO_MM, 0.0]):
        g = np.array(g, dtype=float)
        arm = th.grid_to_arm(g, r_true, t_true)
        assert np.allclose(r_true @ arm + t_true, g, atol=1e-9), \
            'grid = R @ arm + t must hold'
        back = th.arm_to_grid(arm, r_true, t_true)
        assert np.allclose(back, g, atol=1e-9), \
            f'round-trip drift {back - g}'
    # A planar-solve-style R (grid-z column = cross of the in-plane
    # columns — exactly what SVD Procrustes produces for planar data,
    # up to the normal sign) also round-trips exactly.
    u = np.array([1.0, 0.3, -0.2])
    u /= np.linalg.norm(u)
    v = np.array([0.1, 1.0, 0.4])
    v = v - u * (v @ u)
    v /= np.linalg.norm(v)
    r_planar = np.column_stack([u, v, np.cross(u, v)])
    assert abs(np.linalg.det(r_planar) - 1.0) < 1e-9, 'planar R proper'
    g = np.array([50.0, -30.0, 0.0])
    assert np.allclose(
        th.arm_to_grid(th.grid_to_arm(g, r_planar, t_true),
                       r_planar, t_true),
        g, atol=1e-9), 'planar-style R round-trip'
    print('PASS t006-roundtrip: arm->grid->arm exact (< 1e-9 mm), '
          'incl. planar-solve R')


def test_t006_lift():
    """T006 lift math: z_lift=10 changes only the grid-normal
    component by exactly 10 mm, and the signed normal points away from
    the board (positive arm-frame z), flipping a yaml R whose grid +z
    points through the board."""
    rng = np.random.default_rng(23)
    r_true = random_rotation(rng)
    t_true = rng.uniform(-200.0, 200.0, size=3)
    g = np.array([40.0, 60.0, 0.0])
    normal, flipped = th.lift_direction(r_true)
    assert np.isclose(np.linalg.norm(normal), 1.0), 'normal must be unit'
    assert normal[2] > 0.0, 'normal must point away (arm z up)'
    arm0 = th.grid_to_arm(g, r_true, t_true)
    arm_lift = th.target_arm_xyz(g, r_true, t_true, 10.0, normal)
    d = arm_lift - arm0
    assert np.isclose(np.linalg.norm(d), 10.0, atol=1e-9), '10 mm lift'
    assert np.allclose(d, 10.0 * normal, atol=1e-9), \
        'lift must be purely along the grid normal'
    # In the grid frame the lift is exactly +10 in z: the commanded
    # point sits 10 mm above the board plane.
    assert np.allclose(r_true @ arm_lift + t_true,
                       g + [0.0, 0.0, 10.0], atol=1e-9), \
        'grid-frame lift must be exactly [0, 0, 10]'
    # Sign: a yaml R whose grid +z points through the board (downward
    # in the arm frame) is flipped so the lift goes away from it.
    r_down = np.diag([1.0, -1.0, -1.0])  # proper rotation, n0=(0,0,-1)
    normal_d, flipped_d = th.lift_direction(r_down)
    assert flipped_d, 'downward yaml grid z must be flagged flipped'
    assert np.allclose(normal_d, [0.0, 0.0, 1.0]), 'flipped -> up'
    # Zero lift leaves the point unchanged.
    assert np.allclose(th.target_arm_xyz(g, r_true, t_true, 0.0, normal),
                       arm0), 'z_lift=0 must not move the point'
    print('PASS t006-lift: 10 mm = pure grid-normal component; '
          'downward yaml grid-z sign is flipped')


def test_t006_spd():
    """T006 spd clamp: distances 0/50/1000 mm over 2 s -> 50/50/300."""
    f = lambda d: th.compute_spd(d, 2.0, 50.0, 300.0)
    assert f(0.0) == 50.0, '0 mm -> spd-min'
    assert f(50.0) == 50.0, '50 mm -> 25 mm/s clamped to spd-min'
    assert f(1000.0) == 300.0, '1000 mm -> 500 mm/s clamped to spd-max'
    assert f(400.0) == 200.0, '400 mm -> exactly the move-time speed'
    assert th.compute_spd(250.0, 5.0, 50.0, 300.0) == 50.0, \
        '250 mm over 5 s -> 50 (floor)'
    print('PASS t006-spd: 0/50/1000 mm over 2 s -> 50/50/300')


def test_t006_yaml():
    """T006 handeye.yaml loader: reads synthetic solve_handeye.py
    output (PyYAML dump + plain emitter format) and rejects bad
    shapes / a non-rotation R / missing files."""
    rng = np.random.default_rng(31)
    r_true = random_rotation(rng)
    t_true = rng.uniform(-200.0, 200.0, size=3)
    points = [{'target': f't{i}', 'tag_id': 18, 'corner': 'TL',
               'grid': [float(i) / sh.IN_TO_MM, 0.0],
               'arm_xyz_mm': [float(v)
                              for v in rng.uniform(-100.0, 100.0, 3)]}
              for i in range(12)]
    data = sh.handeye_data(points, r_true, t_true, 0.5,
                           np.full(12, 0.5))
    with tempfile.TemporaryDirectory() as tmp:
        pyyaml_path = Path(tmp) / 'pyyaml.yaml'
        plain_path = Path(tmp) / 'plain.yaml'
        sh.dump_yaml(pyyaml_path, data)  # PyYAML when present
        plain_path.write_text(sh.plain_yaml(data))
        for p in (pyyaml_path, plain_path):
            rr, tt = th.load_handeye(p)
            assert np.allclose(rr, r_true, atol=1e-9), f'{p.name}: R'
            assert np.allclose(tt, t_true, atol=1e-9), f'{p.name}: t'
        # The minimal plain-format parser itself (the PyYAML-free
        # fallback path on boxes without PyYAML).
        parsed = th._parse_plain_handeye(sh.plain_yaml(data))
        assert parsed is not None, 'plain format must parse'
        assert np.allclose(parsed['R'], r_true, atol=1e-9)
        assert np.allclose(parsed['t'], t_true, atol=1e-9)
        assert th._parse_plain_handeye('nonsense\n') is None
        # Bad shapes / content are rejected with ValueError.
        bad_r = Path(tmp) / 'bad_r.yaml'
        bad_r.write_text('R: [[1, 0, 0], [0, 1, 0]]\nt: [1, 2, 3]\n')
        bad_det = Path(tmp) / 'bad_det.yaml'
        bad_det.write_text('R: [[1, 0, 0], [0, 1, 0], [0, 0, -1]]\n'
                           't: [1, 2, 3]\n')  # reflection, det=-1
        bad_ortho = Path(tmp) / 'bad_ortho.yaml'
        bad_ortho.write_text('R: [[1, 0, 0], [0, 1, 0.5], [0, 0, 1]]\n'
                             't: [1, 2, 3]\n')  # det=1 but not
                                                # orthonormal
        bad_eq = Path(tmp) / 'bad_eq.yaml'
        bad_eq.write_text(sh.plain_yaml(data).replace(
            'grid_mm = R @ arm_xyz_mm + t',
            'grid_mm = arm_xyz_mm + t'))
        for p in (bad_r, bad_det, bad_ortho, bad_eq):
            try:
                th.load_handeye(p)
                raise AssertionError(f'{p.name} must be rejected')
            except ValueError:
                pass
        try:
            th.load_handeye(Path(tmp) / 'nope.yaml')
            raise AssertionError('missing file must be rejected')
        except ValueError:
            pass
    print('PASS t006-yaml: loader reads solver output (both emitters), '
          'rejects bad R/t/equation/det/orthonormality/missing')


def test_t006_lock():
    """T006 double-run guard: a live pid's lock refuses a second
    acquire; a stale (dead-pid) lock is cleared and re-acquired;
    release removes only our own lock."""
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / 'lock'
        ok, msg = th.acquire_lock(path)
        assert ok, msg
        assert int(path.read_text().strip()) == os.getpid()
        ok2, msg2 = th.acquire_lock(path)
        assert not ok2, 'a second acquire must be refused'
        assert 'refusing' in msg2, 'refusal must explain why'
        th.release_lock(path)
        assert not path.exists(), 'release must remove our lock'
        # Stale lock (dead pid): cleared, then acquired fresh.
        path.write_text('999999999\n')
        ok3, msg3 = th.acquire_lock(path)
        assert ok3, f'stale lock must be cleared: {msg3}'
        assert int(path.read_text().strip()) == os.getpid()
        th.release_lock(path)
        assert not path.exists()
        # A live pid (ours) refuses.
        path.write_text(f'{os.getpid()}\n')
        ok4, _ = th.acquire_lock(path)
        assert not ok4, 'our own live pid must refuse'
        th.release_lock(path)
    print('PASS t006-lock: live pid refused, stale pid cleared, '
          'own lock released')


def test_t006_pipeline():
    """T006 end-to-end core path: a synthetic solver handeye.yaml +
    the REAL grid JSON -> 12 targets -> 12 commanded arm points, each
    exactly z_lift above the board plane in the grid frame, with
    per-move spd inside the clamp bounds."""
    rng = np.random.default_rng(5)
    r_true = random_rotation(rng)
    t_true = rng.uniform(-200.0, 200.0, size=3)
    points = [{'target': f't{i}', 'tag_id': 18, 'corner': 'TL',
               'grid': [float(i) / sh.IN_TO_MM, 0.0],
               'arm_xyz_mm': [float(v)
                              for v in rng.uniform(-100.0, 100.0, 3)]}
              for i in range(12)]
    data = sh.handeye_data(points, r_true, t_true, 0.4, np.full(12, 0.4))
    with tempfile.TemporaryDirectory() as tmp:
        yaml_path = Path(tmp) / 'handeye.yaml'
        sh.dump_yaml(yaml_path, data)
        r, t = th.load_handeye(yaml_path)
        assert np.allclose(r, r_true, atol=1e-9), 'R load mismatch'
        assert np.allclose(t, t_true, atol=1e-9), 't load mismatch'
        targets = th.load_targets(GRID)
        assert len(targets) == 12
        normal, _ = th.lift_direction(r)
        prev = None
        for tg in targets:
            g = np.array([tg['grid'][0], tg['grid'][1], 0.0],
                         dtype=float) * sh.IN_TO_MM
            cmd = th.target_arm_xyz(g, r, t, 10.0, normal)
            back = r @ cmd + t
            assert np.allclose(back[:2], g[:2], atol=1e-9), \
                'lift must not shift the in-plane position'
            assert np.isclose(back[2], 10.0, atol=1e-9), \
                'commanded point must sit 10 mm above the board plane'
            if prev is not None:
                spd = th.compute_spd(np.linalg.norm(cmd - prev),
                                     2.0, 50.0, 300.0)
                assert 50.0 <= spd <= 300.0, f'spd {spd} out of clamp'
            prev = cmd
    print('PASS t006-pipeline: real grid JSON + synthetic yaml -> 12 '
          'lifted commands (grid z == +10 mm, spd clamped)')


def test_py_compile():
    for path in (ARM_BRIDGE, HERE / 'handeye_capture.py',
                 HERE / 'solve_handeye.py', HERE / 'test_handeye.py',
                 HERE / 'selftest.py'):
        assert path.is_file(), f'missing {path}'
        py_compile.compile(str(path), doraise=True)
        print(f'PASS py_compile: {path.name}')


def test_py310_compat():
    # The Orin runs Python 3.10. PEP 701 syntax (same-quote/multiline
    # f-string fields) parses on the Mac's 3.12+ but SyntaxErrors on
    # 3.10 — caught this live on the robot 2026-11-11 (test_handeye.py
    # line 528). Parse every deployed script as 3.10 grammar.
    import ast
    for path in (ARM_BRIDGE, HERE / 'handeye_capture.py',
                 HERE / 'solve_handeye.py', HERE / 'test_handeye.py',
                 HERE / 'selftest.py'):
        source = path.read_text()
        ast.parse(source, filename=str(path), feature_version=(3, 10))
        print(f'PASS py310-grammar: {path.name}')


def main():
    print('T004-T006 selftest (no ROS, no camera, no detector)')
    test_procrustes_exact()
    test_procrustes_noisy()
    test_pairs_path()
    test_targets()
    test_target_dot_ux()
    test_undistort()
    test_t006_roundtrip()
    test_t006_lift()
    test_t006_spd()
    test_t006_yaml()
    test_t006_lock()
    test_t006_pipeline()
    test_py_compile()
    test_py310_compat()
    print('RESULT: PASS — all T004-T006 selftests green')
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except AssertionError as exc:
        print(f'FAIL: {exc}', file=sys.stderr)
        sys.exit(1)
