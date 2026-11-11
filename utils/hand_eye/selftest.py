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
  5. py_compile of arm_bridge.py, handeye_capture.py, solve_handeye.py
     and this file (compile only — no ROS imports are executed)

Exit codes: 0 all pass, 1 a check failed.
"""

import io
import math
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


def test_py_compile():
    for path in (ARM_BRIDGE, HERE / 'handeye_capture.py',
                 HERE / 'solve_handeye.py', HERE / 'selftest.py'):
        assert path.is_file(), f'missing {path}'
        py_compile.compile(str(path), doraise=True)
        print(f'PASS py_compile: {path.name}')


def main():
    print('T004 selftest (no ROS, no camera, no detector)')
    test_procrustes_exact()
    test_procrustes_noisy()
    test_pairs_path()
    test_targets()
    test_undistort()
    test_py_compile()
    print('RESULT: PASS — all T004 selftests green')
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except AssertionError as exc:
        print(f'FAIL: {exc}', file=sys.stderr)
        sys.exit(1)
