#!/usr/bin/env python3
"""T007 — autonomous grasp demo (Orin, ROS2): detect the apriltag grid,
grasp the object on it in end-effector space, and log gripper pressure
and clamp angle to a CSV.

Object placement: the object sits ON the printed 5x4 tag36h11 grid
(IDs 18-37), centered on one of the grid's 12 black corner-squares —
the square in the 3rd row from the top, between the 2nd and 3rd tag
columns. Some tags are occluded by the object; detection uses the
REMAINING visible tags.

TARGET INDEXING ASSUMPTION (verified in the selftest against the real
grid JSON): the grid JSON's "squares" list is row-major — 4 rows of
squares x 3 per row, index = row*3 + col, row 0 = top row, col 0 =
leftmost. The demo square ("3rd row, between 2nd and 3rd columns") is
row 2 (1-indexed row 3), col 1 (middle square column) -> squares[7]
-> center (3.7333, 5.6) in. Params target_row / target_col (defaults
2 / 1) index the same list.

CLAMP ANGLES (hardware-verified from the Orin saved mission
~/arm_waypoints/pick_demo.yaml, joints base/shoulder/elbow/clamp in
rad): POSITIVE = CLOSING — open ≈ 1.859, fully closed ≈ 3.140,
holding grasp ≈ 2.686. CLOSED_REF commands clamp_close_rad (default
3.140) and logs the live read-back angle + pressure; NO later step
computes from it. GRASP is a DIRECT target: the grasp_clamp_rad
param (default 2.686, the pick_demo holding grasp) — the earlier
"closed_ref + 10 deg" rule was dropped in favor of the holding grasp.
clamp_close_rad / clamp_open_rad / grasp_clamp_rad are launch params.

Sequence (one /grasp/start std_srvs/Trigger call = one full run; the
service call BLOCKS until the run finishes):
  1. CLOSED_REF  clamp -> clamp_close_rad (fully closed, default
                 3.140 rad); read the achieved clamp angle back ->
                 closed_ref; log angle + pressure
  2. HOVER       arm/ee_goals -> target + hover_z (30 mm) along the
                 grid normal (away from the board); wait reach
  3. OPEN        clamp -> clamp_open_rad (fully open, default
                 1.859 rad); log
  4. DESCEND     arm/ee_goals -> target + descend_z (10 mm); wait
                 reach + SETTLE (pose stops moving before GRASP)
  5. GRASP       clamp -> grasp_clamp_rad (default 2.686 rad, the
                 holding grasp); log angle + pressure + deltas vs OPEN
  6. LIFT        arm/ee_goals -> target + hover_z; wait reach + settle
  7. HOME        arm/ee_goals -> home_xyz (the hand-captured home,
                 with its yaw); wait reach

EVERY action is timeout-bounded (reach_timeout / clamp_timeout); on
timeout the node logs a WARN naming the stage and PROCEEDS TO THE NEXT
ACTION — an unreachable pose or a clamp stall is expected and the run
never aborts (user requirement).

The arm target comes from the grid JSON + handeye.yaml (board in the
calibration position, exactly like test_handeye.py); the camera only
VERIFIES the grid is in view and reports tag count + target pixel
position. If the grid is not verified at trigger time the run proceeds
with a WARN. Camera: /dev/sentinel_webcam opened directly (no ROS
camera node), 1280x720, C270 dark-start warm-up (read up to 60 frames
discarding mean < 5), undistorted with camera_matrix.yaml (K scaled by
width/height ratio when the capture resolution differs) BEFORE tag
detection.

Pressure: /gripper_pressure_reading (sentinel_msgs/GripperSensors,
uint16 hinge/anvil — raw 12-bit ADC from the motor-controller Pico,
published at 10 Hz by cmd_vel_serial; T007 renamed the topic from the
old "/gripper"). The freshest reading is logged at each stage; a
reading older than 0.5 s is flagged stale.

Safety: torque stays LOCKED for the entire run (no T:210 anywhere in
this node — unlike handeye_capture.py); this node never opens the arm
serial port (arm_bridge owns it); Ctrl+C stops new goals and the arm
holds — an in-flight T:1041 has NO cancel path and completes first
(~2-3 s at the bridge defaults). Keep clear of the arm during the run.

Logging: CSV grasp_log_YYYYmmdd_HHMMSS.csv in output_dir (mkdir -p),
plus a console summary table (stage, clamp rad/deg, hinge, anvil,
deltas vs OPEN).

The core below (grid/homography/undistort/handeye/sequence) is pure
numpy/cv2 — no rclpy — importable on the Mac for the selftest
(test/test_grasp_demo.py). The ROS node class is defined only when the
ROS interfaces import.
"""

import csv
import json
import math
import os
import signal
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

try:
    from dt_apriltags import Detector
except ImportError:  # pragma: no cover — the Orin has it
    Detector = None

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.callback_groups import ReentrantCallbackGroup
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import JointState
    from std_srvs.srv import Trigger
    from sentinel_msgs.msg import GripperSensors
    HAVE_ROS = True
except ImportError:  # pragma: no cover — Mac selftest has no ROS
    HAVE_ROS = False

IN_TO_MM = 25.4
CLAMP_JOINT = 'clamp'
STAGES = ('CLOSED_REF', 'HOVER', 'OPEN', 'DESCEND', 'GRASP', 'LIFT',
          'HOME')

DARK_THRESHOLD = 5.0
QUIET_ZONE_FRACTION = 0.8
MIN_TAGS_FOR_TARGET = 4
POSE_MAX_AGE_S = 1.0        # a fresh arm/ee_pose is younger than this
PRESSURE_MAX_AGE_S = 0.5    # older /gripper readings are flagged stale
NORMAL_Z_WARN = 0.2         # |normal.arm_z| below this: sign check shaky
CAM_RETRY_EVERY_S = 5.0
STATUS_EVERY_S = 2.0
LOCK_PATH = '/tmp/grasp_demo.lock'

# Settle check: hover and descend targets are only ~20 mm apart — inside
# reach_tol — so a plain wait_reach would pass on the first fresh sample
# while the arm is still moving. wait_settled instead requires N
# consecutive FRESH samples that are inside the tolerance AND moved less
# than SETTLE_DELTA_MM since the previous sample.
SETTLE_N_SAMPLES = 5
SETTLE_DELTA_MM = 2.0

DETECT_MAX_AGE_S = 5.0      # grid verification older than this: stale

# Deployed-Orin defaults (Sentinel/ work root, per user rule 2026-11-13).
DEFAULT_GRID_JSON = ('/home/sentinel1/dean/Sentinel/hand_eye/'
                     'grid_5x4_tag36h11_ids18-37.json')
DEFAULT_HANDEYE = '/home/sentinel1/dean/Sentinel/hand_eye/handeye.yaml'
DEFAULT_CAMERA_MATRIX = ('/home/sentinel1/dean/Sentinel/camera_calibration/'
                         'camera_matrix.yaml')
DEFAULT_CAMERA_DEVICE = '/dev/sentinel_webcam'
DEFAULT_OUTPUT_DIR = '/home/sentinel1/dean/Sentinel/grasp_demo'
# Home pose: EE XYZ (mm) + t (rad), hand-captured with torque off
# (2026-10-23) — the SAME home the full bringup uses (arm 'home' param
# in sentinel_bringup.launch.py). The demo's final HOME stage returns
# the arm here.
DEFAULT_HOME_XYZ = [3.28, -47.44, 69.38, 3.134]


# ---------------------------------------------------------------------
# Grid / homography / undistort core — copied from handeye_capture.py
# (T004; do not edit the original there). No ROS, no camera.
# ---------------------------------------------------------------------

def load_grid_data(json_path):
    """Parsed grid JSON dict (meta + tags + squares)."""
    return json.loads(Path(json_path).read_text())


def load_grid(json_path):
    """Return {tag_id: (cx_in, cy_in, side_in)} from the grid JSON."""
    data = load_grid_data(json_path)
    meta = data.get('meta', {})
    if not meta.get('size_in_includes_quiet_zone', False):
        raise ValueError(
            f'{json_path}: meta.size_in_includes_quiet_zone must be true')
    return {t['id']: (t['center']['x_in'], t['center']['y_in'],
                      t['size_in']) for t in data['tags']}


def tag_object_corners(cx, cy, side, frac=QUIET_ZONE_FRACTION):
    """The 4 black-border corners of a tag, order TL, TR, BR, BL (in)."""
    h = frac * side / 2.0
    return [(cx - h, cy - h), (cx + h, cy - h),
            (cx + h, cy + h), (cx - h, cy + h)]


def fit_homography(src, dst):
    """Least-squares homography mapping src -> dst (N >= 4), DLT via SVD."""
    n = len(src)
    a = np.zeros((2 * n, 9))
    for i, ((x, y), (u, v)) in enumerate(zip(src, dst)):
        a[2 * i] = [x, y, 1, 0, 0, 0, -u * x, -u * y, -u]
        a[2 * i + 1] = [0, 0, 0, x, y, 1, -v * x, -v * y, -v]
    _, _, vt = np.linalg.svd(a)
    hh = vt[-1].reshape(3, 3)
    return hh / hh[2, 2]


def apply_homography(hh, pts):
    """Apply a homography to (N,2) points, returning (N,2)."""
    pts = np.atleast_2d(np.asarray(pts, dtype=float))
    t = hh @ np.column_stack([pts, np.ones(len(pts))]).T
    return (t[:2] / t[2]).T


def detection_fields(d):
    """(tag_id, corners (4,2), center (2,)) in a detector-agnostic shape."""
    tid = int(d.tag_id)
    corners = np.asarray(d.corners, dtype=float).reshape(-1, 2)
    center = np.asarray(d.center, dtype=float).ravel()[:2]
    return tid, corners, center


def match_corners(grid_tags, detections, min_tags=MIN_TAGS_FOR_TARGET):
    """Match detected tag corners to grid object corners (T003 method).

    Returns (obj (M,2) in, img (M,2) px, n_tags): black-border corners
    at 0.8 x side, nearest-corner assignment per tag through a
    center-fit homography (convention-free), or (None, None, n) when
    fewer than min_tags tags matched / usable.
    """
    det_by_id = {}
    for d in detections:
        tid, _, _ = detection_fields(d)
        if tid in grid_tags and tid not in det_by_id:
            det_by_id[tid] = d
    ids = sorted(det_by_id)
    if len(ids) < min_tags:
        return None, None, len(ids)
    src = np.array([grid_tags[i][:2] for i in ids], dtype=float)
    dst = np.array([detection_fields(det_by_id[i])[2] for i in ids],
                   dtype=float)
    hh = fit_homography(src, dst)
    obj_list, img_list = [], []
    for i in ids:
        cx, cy, side = grid_tags[i]
        corners_obj = np.asarray(tag_object_corners(cx, cy, side),
                                 dtype=float)
        _, corners_img, _ = detection_fields(det_by_id[i])
        proj = apply_homography(hh, corners_obj)
        order = [int(np.argmin(np.linalg.norm(corners_img - p, axis=1)))
                 for p in proj]
        if len(set(order)) != 4:
            continue  # ambiguous assignment; skip this tag
        obj_list.append(corners_obj)
        img_list.append(corners_img[order])
    if len(obj_list) < min_tags:
        return None, None, len(obj_list)
    return np.vstack(obj_list), np.vstack(img_list), len(obj_list)


def center_homography(grid_tags, detections, min_tags=MIN_TAGS_FOR_TARGET):
    """Fallback H from tag centers only. Returns (hh, n_tags) or None."""
    det_by_id = {}
    for d in detections:
        tid, _, _ = detection_fields(d)
        if tid in grid_tags and tid not in det_by_id:
            det_by_id[tid] = d
    ids = sorted(det_by_id)
    if len(ids) < min_tags:
        return None, len(ids)
    src = np.array([grid_tags[i][:2] for i in ids], dtype=float)
    dst = np.array([detection_fields(det_by_id[i])[2] for i in ids],
                   dtype=float)
    return fit_homography(src, dst), len(ids)


def load_camera_matrix(path):
    """Load (camera_matrix, dist_coeffs, image_size) from a T003
    camera_matrix.yaml (image_size may be None when absent).

    Returns None when the file is missing, unreadable, or lacks the
    keys — with a WARN on stderr (the caller then continues with raw
    frames)."""
    p = Path(path)
    if not p.is_file():
        print(f'WARN: camera matrix {p} not found — continuing with RAW '
              'frames (undistort off)', file=sys.stderr)
        return None
    try:
        import yaml
        data = yaml.safe_load(p.read_text())
        if not isinstance(data, dict) or 'camera_matrix' not in data \
                or 'dist_coeffs' not in data:
            raise ValueError('missing camera_matrix/dist_coeffs keys')
        k = np.asarray(data['camera_matrix'], dtype=float).reshape(3, 3)
        dist = np.asarray(data['dist_coeffs'], dtype=float).ravel()
        size = data.get('image_size')
        if size is not None:
            size = (int(size[0]), int(size[1]))
        return k, dist, size
    except (OSError, ValueError, ImportError, TypeError) as exc:
        print(f'WARN: cannot read {p}: {exc} — continuing with RAW '
              'frames (undistort off)', file=sys.stderr)
        return None


def scale_intrinsics(k, calib_wh, capture_wh):
    """Scale a calibration K to a different capture resolution.

    Standard practice: fx and cx scale with the width ratio, fy and cy
    with the height ratio. Returns the same K when the sizes match."""
    cw, ch = calib_wh
    w, h = capture_wh
    sx, sy = float(w) / float(cw), float(h) / float(ch)
    kk = k.copy()
    kk[0, 0] *= sx
    kk[0, 2] *= sx
    kk[1, 1] *= sy
    kk[1, 2] *= sy
    return kk


def build_undistort_maps(k, dist, size_wh):
    """initUndistortRectifyMap for a (w, h) frame; returns (map1, map2)."""
    w, h = int(size_wh[0]), int(size_wh[1])
    m1, m2 = cv2.initUndistortRectifyMap(
        k, dist, None, k, (w, h), cv2.CV_32FC1)
    return m1, m2


def undistort_frame(frame, maps):
    """Remap a frame through the precomputed undistort maps."""
    return cv2.remap(frame, maps[0], maps[1], cv2.INTER_LINEAR)


def warm_up(cap, max_reads=60, dark_thr=DARK_THRESHOLD):
    """Read frames until a bright one arrives (C270 ramp); or None."""
    for _ in range(max_reads):
        ok, frame = cap.read()
        if not ok:
            continue
        if float(frame.mean()) >= dark_thr:
            return frame
    return None


# ---------------------------------------------------------------------
# Hand-eye core — copied from test_handeye.py (T006; do not edit the
# original there). No ROS.
# ---------------------------------------------------------------------

def _parse_plain_handeye(text):
    """Minimal parser for solve_handeye.py plain_yaml() output: an 'R:'
    section of '  - [a, b, c]' rows and a 't: [a, b, c]' line. Returns
    {'R': [...], 't': [...]} or None (unrecognized shape)."""
    rows = []
    t = None
    in_r = False
    for raw in text.splitlines():
        line = raw.strip()
        if line == 'R:':
            in_r = True
            continue
        if in_r and line.startswith('- ['):
            try:
                rows.append([float(v) for v in
                             line[2:].strip().strip('[]').split(',')])
            except ValueError:
                return None
        elif line.startswith('t:'):
            try:
                t = [float(v) for v in
                     line[2:].strip().strip('[]').split(',')]
            except ValueError:
                return None
    if len(rows) != 3 or t is None:
        return None
    return {'R': rows, 't': t}


def load_handeye(path):
    """(R (3,3), t (3,)) from a solve_handeye.py handeye.yaml.

    Accepts both the PyYAML dump and the built-in plain emitter output.
    Raises ValueError on a missing/unreadable file, a bad R/t shape, a
    non-proper-rotation R, or an unexpected 'equation' field."""
    p = Path(path)
    if not p.is_file():
        raise ValueError(f'handeye.yaml not found: {p}')
    text = p.read_text()
    data = None
    try:
        import yaml
    except ImportError:  # pragma: no cover — ROS distros ship PyYAML
        yaml = None
    if yaml is not None:
        try:
            data = yaml.safe_load(text)
        except yaml.YAMLError:
            data = None
    if data is None:
        data = _parse_plain_handeye(text)
        if data is None:
            raise ValueError(f'cannot parse {p} (not PyYAML-safe and '
                             'not the plain emitter format)')
    if not isinstance(data, dict) or 'R' not in data or 't' not in data:
        raise ValueError(f'{p}: needs R and t keys '
                         '(solve_handeye.py output)')
    r = np.asarray(data['R'], dtype=float)
    t = np.asarray(data['t'], dtype=float)
    if r.shape != (3, 3):
        raise ValueError(f'{p}: R must be 3x3, got {r.shape}')
    if t.shape != (3,):
        raise ValueError(f'{p}: t must be length 3, got {t.shape}')
    det = float(np.linalg.det(r))
    if abs(det - 1.0) > 1e-3:
        raise ValueError(f'{p}: R is not a proper rotation '
                         f'(det={det:.3f}) — bad handeye.yaml?')
    if not np.allclose(r.T @ r, np.eye(3), atol=1e-3):
        raise ValueError(f'{p}: R is not orthonormal (R^T @ R != I) — '
                         'bad handeye.yaml?')
    equation = data.get('equation')
    if equation is not None and equation != 'grid_mm = R @ arm_xyz_mm + t':
        raise ValueError(f'{p}: unexpected equation {equation!r}')
    return r, t


def grid_to_arm(grid_mm, r, t):
    """grid (3,) mm -> arm (3,) mm: arm = R^T @ (grid - t)."""
    g = np.asarray(grid_mm, dtype=float)
    return (np.asarray(r, dtype=float).T
            @ (g - np.asarray(t, dtype=float))).reshape(3)


def arm_to_grid(arm_mm, r, t):
    """arm (3,) mm -> grid (3,) mm: grid = R @ arm + t (handeye eq.)."""
    a = np.asarray(arm_mm, dtype=float)
    return (np.asarray(r, dtype=float) @ a
            + np.asarray(t, dtype=float)).reshape(3)


def lift_direction(r):
    """Grid +z axis in the arm frame, signed so +z_lift moves the tip
    AWAY from the board. Returns (n (3,) unit, flipped bool).

    The grid frame is 2D on the board (x right, y down, z = 0 in the
    plane — T004). The grid +z direction in arm space is n0 = R^T @
    [0,0,1]. The planar Procrustes solve pins n0 only up to SIGN; the
    physical setup disambiguates: the arm reaches the board from above
    (arm frame x forward / y left / z UP per the bridge's T:1041
    convention), so "away from the board" must have a positive
    arm-frame z component. If n0 points downward (n0[2] < 0) we flip
    it. When |n0[2]| is small (board not below the arm) the check is
    unreliable — we warn instead of guessing silently."""
    n0 = np.asarray(r, dtype=float).T @ np.array([0.0, 0.0, 1.0])
    n0 = n0 / np.linalg.norm(n0)
    if abs(n0[2]) < NORMAL_Z_WARN:
        print(f'WARN: grid normal has a small arm-z component '
              f'({n0[2]:.3f}) — the sign check assumes the arm is '
              'above the board; verify the printed lift direction '
              'before the run!', file=sys.stderr)
    if n0[2] < 0.0:
        return -n0, True
    return n0, False


# ---------------------------------------------------------------------
# T007 core: target square, sequence, logging — no ROS.
# ---------------------------------------------------------------------

def target_square_center(grid_data, row, col):
    """Center (x_in, y_in) of the black corner-square at (row, col),
    0-indexed, row-major exactly as the grid JSON's squares list is
    ordered (row 0 = top row of squares, col 0 = leftmost). The demo
    target (row 2, col 1) is squares[7] = (3.7333, 5.6) in — the 3rd
    row from the top, between the 2nd and 3rd tag columns."""
    squares = grid_data['squares']
    row = int(row)
    col = int(col)
    idx = row * 3 + col
    if row < 0 or col < 0 or col > 2 or idx >= len(squares):
        raise ValueError(
            f'target square row {row} col {col} out of range '
            f'(squares list has {len(squares)} entries, 3 per row)')
    sq = squares[idx]
    return float(sq['center']['x_in']), float(sq['center']['y_in'])


def deg_to_rad(deg):
    return math.radians(float(deg))


def quat_from_yaw(yaw):
    """(w, z) of the euler (0, 0, yaw) quaternion — the bridge's T:1041
    yaw convention (w = cos(yaw/2), z = sin(yaw/2))."""
    return math.cos(yaw / 2.0), math.sin(yaw / 2.0)


class SettleTracker:
    """Pose-settle detector (pure — no ROS). Settled only after N
    consecutive FRESH samples that are BOTH inside the tolerance and
    moved less than delta_mm since the previous sample. A plain
    err <= tol check is vacuous for DESCEND/LIFT: hover and descend
    targets are only ~20 mm apart (inside reach_tol), so the first
    fresh sample reads "reached" while the arm is still moving. The
    node feeds fresh arm/ee_pose samples inside its nested spin; the
    selftest feeds simulated samples."""

    def __init__(self, tol, delta_mm=SETTLE_DELTA_MM,
                 n=SETTLE_N_SAMPLES):
        self.tol = float(tol)
        self.delta = float(delta_mm)
        self.n = int(n)
        self.prev = None
        self.count = 0

    def feed(self, xyz, cmd):
        """One fresh sample. Returns (settled, err_mm, note). A stale
        gap must reset the run: call reset() when no fresh pose exists
        (consecutive samples only count when FRESH)."""
        p = np.asarray(xyz, dtype=float)
        c = np.asarray(cmd, dtype=float)
        err = float(np.linalg.norm(p - c))
        moved = None if self.prev is None else \
            float(np.linalg.norm(p - self.prev))
        self.prev = p
        if err <= self.tol and moved is not None and moved < self.delta:
            self.count += 1
            if self.count >= self.n:
                return True, err, f'settled, err {err:.1f} mm'
            return False, err, \
                f'settling {self.count}/{self.n}, err {err:.1f} mm'
        self.count = 0
        if moved is None:
            note = f'first sample, err {err:.1f} mm'
        elif err > self.tol:
            note = f'err {err:.1f} mm > tol'
        else:
            note = f'still moving ({moved:.1f} mm/sample)'
        return False, err, note

    def reset(self):
        self.prev = None
        self.count = 0


CSV_FIELDS = ('stage', 'clamp_angle_rad', 'clamp_angle_deg', 'hinge',
              'anvil', 'delta_clamp_deg', 'delta_hinge',
              'delta_anvil', 'note')


def new_row(stage, clamp_rad, hinge, anvil, note):
    """One log row: clamp angle rad + rounded deg, pressure counts.
    Delta columns stay empty until attach_deltas fills them (GRASP)."""
    return {'stage': stage,
            'clamp_angle_rad': (round(float(clamp_rad), 6)
                                if clamp_rad is not None else None),
            'clamp_angle_deg': (round(math.degrees(float(clamp_rad)), 3)
                                if clamp_rad is not None else None),
            'hinge': hinge, 'anvil': anvil,
            'delta_clamp_deg': None, 'delta_hinge': None,
            'delta_anvil': None,
            'note': note or ''}


def attach_deltas(row, base_row):
    """GRASP deltas vs the OPEN row (deg for the angle, raw counts for
    the pressure channels). Left None where either side is missing."""
    if base_row is None:
        return
    if row['clamp_angle_deg'] is not None and \
            base_row['clamp_angle_deg'] is not None:
        row['delta_clamp_deg'] = round(
            row['clamp_angle_deg'] - base_row['clamp_angle_deg'], 3)
    for key in ('hinge', 'anvil'):
        if row[key] is not None and base_row[key] is not None:
            row['delta_' + key] = row[key] - base_row[key]


def write_log_csv(path, rows):
    """Write the run rows to CSV (columns = CSV_FIELDS). Returns path."""
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(CSV_FIELDS)
        for r in rows:
            w.writerow([r[field] for field in CSV_FIELDS])
    return path


def summarize(rows):
    """Console summary table: stage, clamp deg, hinge, anvil, deltas."""
    out = [f'{"stage":<12}{"clamp_deg":>10}{"hinge":>7}{"anvil":>7}'
           f'{"d_clamp_deg":>12}{"d_hinge":>9}{"d_anvil":>9}  note']
    for r in rows:
        def s(key):
            v = r[key]
            return '-' if v is None else str(v)
        out.append(f'{r["stage"]:<12}{s("clamp_angle_deg"):>10}'
                   f'{s("hinge"):>7}{s("anvil"):>7}'
                   f'{s("delta_clamp_deg"):>12}{s("delta_hinge"):>9}'
                   f'{s("delta_anvil"):>9}  {r["note"]}')
    return '\n'.join(out)


def _join_notes(*notes):
    return '; '.join(n for n in notes if n)


def run_sequence(env, target_arm, normal, params, logger):
    """The 7-stage grasp sequence (pure — no ROS). Returns the rows.

    env duck-type (the node implements it; the selftest uses a fake):
      publish_joint_clamp(rad)                       -> None
      publish_ee_goal(xyz_mm, yaw_rad=None)          -> None
      wait_reach(xyz_mm, timeout_s) -> (reached, err_mm, note)
      wait_settled(xyz_mm, timeout_s) -> (settled, err_mm, note)
      wait_clamp(rad, timeout_s)    -> (reached, angle_rad, note)
      read_pressure()               -> (hinge, anvil, note)
      read_clamp()                  -> (angle_rad, note)
    params: clamp_close_rad, clamp_open_rad, clamp_tol, grasp_clamp_rad,
            hover_z, descend_z, reach_tol, reach_timeout, clamp_timeout,
            home_xyz ([x, y, z, t])
    logger: info(msg) / warn(msg).

    NEVER raises for a motion/clamp failure: every action is timeout-
    bounded; a failed action logs a WARN naming the stage and the run
    moves to the NEXT action (user requirement: never abort the run).
    """
    rows = []
    open_row = None

    def log_stage(stage, clamp, press, note):
        row = new_row(stage, clamp, press[0], press[1], note)
        rows.append(row)
        return row

    def ee_stage(stage, cmd, yaw=None, settle=False):
        """One timeout-bounded EE move + reach wait (+ settle check for
        DESCEND/LIFT) + log; a timeout WARNs and the run continues to
        the next action."""
        logger.info(f'{stage}: arm/ee_goals -> ({cmd[0]:.1f}, '
                    f'{cmd[1]:.1f}, {cmd[2]:.1f}) mm')
        env.publish_ee_goal(cmd, yaw_rad=yaw)
        reached, err, note = env.wait_reach(cmd,
                                            float(params['reach_timeout']))
        if not reached:
            logger.warn(f'{stage}: not reached in '
                        f'{params["reach_timeout"]:.1f} s ({note}) — '
                        'continuing to the next action')
        if settle:
            # DESCEND/LIFT: wait_reach alone is vacuous (hover and
            # descend targets are ~20 mm apart, inside reach_tol) —
            # require the pose to actually stop moving before GRASP.
            settled, serr, snote = env.wait_settled(
                cmd, float(params['reach_timeout']))
            if not settled:
                logger.warn(f'{stage}: not settled in '
                            f'{params["reach_timeout"]:.1f} s ({snote}) '
                            '— continuing to the next action')
            note = _join_notes(note, snote)
        clamp, cnote = env.read_clamp()
        press = env.read_pressure()
        log_stage(stage, clamp, press, _join_notes(note, cnote, press[2]))

    # -- 1. CLOSED_REF: command fully closed (hardware-verified stop),
    #       read back the achieved angle + log it with the pressure.
    #       No later step computes from closed_ref (the GRASP target is
    #       the direct grasp_clamp_rad param).
    close_cmd = float(params['clamp_close_rad'])
    logger.info(f'CLOSED_REF: clamp -> {close_cmd:.3f} rad '
                '(fully closed)')
    env.publish_joint_clamp(close_cmd)
    reached, angle, note = env.wait_clamp(close_cmd,
                                          float(params['clamp_timeout']))
    closed_ref = angle if angle is not None else close_cmd
    if not reached:
        logger.warn(
            f'CLOSED_REF: clamp not at {close_cmd:.3f} rad in '
            f'{params["clamp_timeout"]:.1f} s (read back '
            f'{closed_ref:.3f} rad) — closed reading is suspect; '
            'continuing to the next action')
    press = env.read_pressure()
    log_stage('CLOSED_REF', angle, press, _join_notes(note, press[2]))
    logger.info(f'CLOSED_REF: closed_ref = {closed_ref:.3f} rad '
                f'({math.degrees(closed_ref):.1f} deg)')

    # -- 2. HOVER: target + hover_z along the grid normal (away from
    #       the board), current gripper yaw passes through.
    target = np.asarray(target_arm, dtype=float)
    n = np.asarray(normal, dtype=float)
    hover_cmd = target + float(params['hover_z']) * n
    ee_stage('HOVER', hover_cmd)

    # -- 3. OPEN: clamp fully open, then log.
    open_cmd = float(params['clamp_open_rad'])
    logger.info(f'OPEN: clamp -> {open_cmd:.3f} rad (fully open)')
    env.publish_joint_clamp(open_cmd)
    reached, angle, note = env.wait_clamp(open_cmd,
                                          float(params['clamp_timeout']))
    if not reached:
        logger.warn(f'OPEN: clamp not at {open_cmd:.3f} rad in '
                    f'{params["clamp_timeout"]:.1f} s — continuing to '
                    'the next action')
    press = env.read_pressure()
    open_row = log_stage('OPEN', angle, press, _join_notes(note, press[2]))

    # -- 4. DESCEND: target + descend_z.
    descend_cmd = target + float(params['descend_z']) * n
    ee_stage('DESCEND', descend_cmd, settle=True)

    # -- 5. GRASP: DIRECT target = grasp_clamp_rad (default 2.686 rad,
    #       the hardware-verified holding grasp from pick_demo.yaml);
    #       log deltas vs OPEN.
    grasp_cmd = float(params['grasp_clamp_rad'])
    logger.info(f'GRASP: clamp -> {grasp_cmd:.3f} rad (holding grasp)')
    env.publish_joint_clamp(grasp_cmd)
    reached, angle, note = env.wait_clamp(grasp_cmd,
                                          float(params['clamp_timeout']))
    if not reached:
        logger.warn('GRASP: clamp stall (read back '
                    f'{angle if angle is not None else "n/a"} rad) — '
                    'grasp may be incomplete; continuing')
    press = env.read_pressure()
    row = log_stage('GRASP', angle, press, _join_notes(note, press[2]))
    attach_deltas(row, open_row)

    # -- 6. LIFT: back up to hover height, then ...
    ee_stage('LIFT', hover_cmd, settle=True)

    # -- 7. HOME: the hand-captured home pose with its yaw.
    home = params['home_xyz']
    home_xyz = np.array([float(home[0]), float(home[1]), float(home[2])])
    ee_stage('HOME', home_xyz, yaw=float(home[3]))

    return rows


# ---------------------------------------------------------------------
# Single-instance guard (two runs would interleave goal streams on one
# arm) — copied pattern from test_handeye.py.
# ---------------------------------------------------------------------

def pid_alive(pid):
    """True when a process with this pid exists (kill(pid, 0) probe)."""
    try:
        os.kill(pid, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:  # pragma: no cover — exists, not ours
        return True


def acquire_lock(path=LOCK_PATH):
    """Refuse when another LIVE instance holds the lock; clear stale
    (dead-pid) locks; write our pid. Returns (ok, message)."""
    p = Path(path)
    if p.exists():
        old_pid = None
        try:
            old_pid = int(p.read_text().strip())
        except (OSError, ValueError):
            pass
        if old_pid is not None and pid_alive(old_pid):
            return False, (f'another grasp_demo.py (pid {old_pid}) '
                           f'holds {p} — refusing to start a second run')
        try:
            p.unlink()  # stale lock: the holder is gone
        except OSError:  # pragma: no cover
            pass
    try:
        p.write_text(f'{os.getpid()}\n')
    except OSError as exc:  # pragma: no cover
        return False, f'cannot create lock {p}: {exc}'
    return True, 'ok'


def release_lock(path=LOCK_PATH):
    """Remove the lock only if it still holds OUR pid (never delete a
    lock a newer instance may have re-created)."""
    p = Path(path)
    try:
        if int(p.read_text().strip()) == os.getpid():
            p.unlink()
    except (OSError, ValueError):
        pass


# ---------------------------------------------------------------------
# ROS node (defined only when the ROS interfaces import)
# ---------------------------------------------------------------------

if HAVE_ROS:  # pragma: no cover — runs on the Orin with ROS2

    class GraspDemoNode(Node):

        def __init__(self):
            super().__init__('grasp_demo')
            self.declare_parameter('grid_json', DEFAULT_GRID_JSON)
            self.declare_parameter('handeye_yaml', DEFAULT_HANDEYE)
            self.declare_parameter('camera_matrix_yaml',
                                   DEFAULT_CAMERA_MATRIX)
            self.declare_parameter('camera_device', DEFAULT_CAMERA_DEVICE)
            self.declare_parameter('camera_width', 1280)
            self.declare_parameter('camera_height', 720)
            self.declare_parameter('output_dir', DEFAULT_OUTPUT_DIR)
            self.declare_parameter('target_row', 2)
            self.declare_parameter('target_col', 1)
            self.declare_parameter('hover_z', 30.0)
            self.declare_parameter('descend_z', 10.0)
            self.declare_parameter('grasp_clamp_rad', 2.686)
            self.declare_parameter('reach_tol', 25.0)
            self.declare_parameter('reach_timeout', 10.0)
            self.declare_parameter('clamp_timeout', 5.0)
            self.declare_parameter('clamp_tol', 0.05)
            self.declare_parameter('clamp_close_rad', 3.140)
            self.declare_parameter('clamp_open_rad', 1.859)
            self.declare_parameter('home_xyz', DEFAULT_HOME_XYZ)
            self.declare_parameter('min_tags', MIN_TAGS_FOR_TARGET)
            self.declare_parameter('detect_rate', 5.0)

            self.params = {
                'hover_z': float(self.get_parameter('hover_z').value),
                'descend_z': float(self.get_parameter('descend_z').value),
                'grasp_clamp_rad':
                    float(self.get_parameter('grasp_clamp_rad').value),
                'reach_tol': float(self.get_parameter('reach_tol').value),
                'reach_timeout':
                    float(self.get_parameter('reach_timeout').value),
                'clamp_timeout':
                    float(self.get_parameter('clamp_timeout').value),
                'clamp_tol': float(self.get_parameter('clamp_tol').value),
                'clamp_close_rad':
                    float(self.get_parameter('clamp_close_rad').value),
                'clamp_open_rad':
                    float(self.get_parameter('clamp_open_rad').value),
                'home_xyz': self._home_xyz_param(),
                'min_tags': int(self.get_parameter('min_tags').value),
            }
            self.output_dir = self.get_parameter('output_dir').value
            self.camera_device = self.get_parameter('camera_device').value
            self.camera_matrix_yaml = Path(
                self.get_parameter('camera_matrix_yaml').value).expanduser()
            self.camera_width = \
                int(self.get_parameter('camera_width').value)
            self.camera_height = \
                int(self.get_parameter('camera_height').value)
            self.detect_rate = \
                float(self.get_parameter('detect_rate').value)

            # --- feedback state ---
            self.ee_pose = None       # latest PoseStamped
            self.ee_time = None
            self.joint_pos = {}       # name -> rad, latest
            self.joint_time = None
            self.gripper = None       # latest GripperSensors
            self.gripper_time = None

            self._ee_sub = self.create_subscription(
                PoseStamped, 'arm/ee_pose', self._ee_cb, 10)
            self._js_sub = self.create_subscription(
                JointState, 'arm/joint_states', self._js_cb, 10)
            self._gripper_sub = self.create_subscription(
                GripperSensors, 'gripper_pressure_reading',
                self._gripper_cb, 10)
            self._goal_pub = self.create_publisher(
                PoseStamped, 'arm/ee_goals', 10)
            self._joint_goal_pub = self.create_publisher(
                JointState, 'arm/joint_goals', 10)

            # The service callback runs the whole sequence synchronously
            # and nested-spins (rclpy.spin_once) while waiting, so the
            # detection timer and the feedback subscriptions must be able
            # to execute while it runs: the service lives in a reentrant
            # group (it holds no default-group lock), everything else in
            # the default group (never concurrent with itself).
            self._start_group = ReentrantCallbackGroup()
            self._start_srv = self.create_service(
                Trigger, 'grasp/start', self._start_cb,
                callback_group=self._start_group)
            self._busy = False
            self._last_csv = None
            self._tick_timer = self.create_timer(
                1.0 / self.detect_rate, self._tick)

            # --- camera / detection state ---
            self.cap = None
            self._cam_attempt_time = 0.0
            self._warmed = False
            self.maps = None
            self.detector = Detector(families='tag36h11',
                                     quad_decimate=1.0) \
                if Detector is not None else None
            self._no_detector_warned = False
            self.grid_tags = {}
            self.target_in = None
            self.target_arm = None
            self.normal = None
            self.target_px = None
            self.last_tags = 0
            self.detect_time = None
            self._degraded = False
            self._last_status = 0.0

            self._load_target()
            self.get_logger().info(
                'grasp_demo ready — trigger with: '
                'ros2 service call /grasp/start std_srvs/srv/Trigger '
                '\'{}\'')

        # --- target / normal from grid JSON + handeye.yaml ---

        def _home_xyz_param(self):
            v = self.get_parameter('home_xyz').value
            try:
                out = [float(x) for x in v]
                if len(out) != 4:
                    raise ValueError
                return out
            except (TypeError, ValueError):
                self.get_logger().warn(
                    f'home_xyz {v!r} is not [x, y, z, t] — falling back '
                    f'to {DEFAULT_HOME_XYZ}')
                return list(DEFAULT_HOME_XYZ)

        def _load_target(self):
            grid_json = Path(
                self.get_parameter('grid_json').value).expanduser()
            handeye_yaml = Path(
                self.get_parameter('handeye_yaml').value).expanduser()
            try:
                data = load_grid_data(grid_json)
                self.grid_tags = load_grid(grid_json)
                row = int(self.get_parameter('target_row').value)
                col = int(self.get_parameter('target_col').value)
                self.target_in = target_square_center(data, row, col)
                self.get_logger().info(
                    f'target square row {row} col {col} '
                    '(0-indexed, row-major) = '
                    f'({self.target_in[0]:.4f}, {self.target_in[1]:.4f}) '
                    'in — 3rd row from top, middle square column')
            except (OSError, ValueError, KeyError,
                    json.JSONDecodeError) as exc:
                self.grid_tags = {}
                self.target_in = None
                self.get_logger().warn(
                    f'grid JSON problem ({grid_json}): {exc} — detection '
                    'off and no target; the sequence will refuse to start')
            try:
                r, t = load_handeye(handeye_yaml)
            except ValueError as exc:
                self.get_logger().warn(
                    f'handeye.yaml problem ({handeye_yaml}): {exc} — the '
                    'sequence will refuse to start')
                return
            if self.target_in is None:
                return
            g = np.array([self.target_in[0], self.target_in[1], 0.0],
                         dtype=float) * IN_TO_MM
            self.target_arm = grid_to_arm(g, r, t)
            self.normal, flipped = lift_direction(r)
            if flipped:
                self.get_logger().warn(
                    'grid normal sign FLIPPED (yaml grid z pointed '
                    'through the board)')
            if abs(self.normal[2]) < NORMAL_Z_WARN:
                self.get_logger().warn(
                    f'grid normal has a small arm-z component '
                    f'({self.normal[2]:.3f}) — verify the lift direction '
                    'before triggering!')
            hover = self.target_arm + self.params['hover_z'] * self.normal
            descend = self.target_arm + \
                self.params['descend_z'] * self.normal
            self.get_logger().info(
                f'target arm XYZ ({self.target_arm[0]:.1f}, '
                f'{self.target_arm[1]:.1f}, {self.target_arm[2]:.1f}) '
                f'mm; normal ({self.normal[0]:+.3f}, '
                f'{self.normal[1]:+.3f}, {self.normal[2]:+.3f}) arm '
                f'frame; hover ({hover[0]:.1f}, {hover[1]:.1f}, '
                f'{hover[2]:.1f}) mm; descend ({descend[0]:.1f}, '
                f'{descend[1]:.1f}, {descend[2]:.1f}) mm')

        # --- feedback callbacks ---

        def _ee_cb(self, msg):
            self.ee_pose = msg
            self.ee_time = time.time()

        def _js_cb(self, msg):
            self.joint_time = time.time()
            self.joint_pos = dict(zip(msg.name, msg.position))

        def _gripper_cb(self, msg):
            self.gripper = msg
            self.gripper_time = time.time()

        def ee_age(self):
            return None if self.ee_time is None \
                else time.time() - self.ee_time

        def fresh_ee_xyz(self):
            """Latest FRESH arm/ee_pose as (x, y, z) mm, or None."""
            if self.ee_pose is None or self.ee_age() > POSE_MAX_AGE_S:
                return None
            p = self.ee_pose.pose.position
            return np.array([p.x, p.y, p.z], dtype=float)

        # --- sequence env (the duck-typed interface run_sequence uses) ---

        def publish_joint_clamp(self, rad):
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = [CLAMP_JOINT]
            msg.position = [float(rad)]
            self._joint_goal_pub.publish(msg)

        def publish_ee_goal(self, xyz, yaw_rad=None):
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'arm_base_link'
            msg.pose.position.x = float(xyz[0])
            msg.pose.position.y = float(xyz[1])
            msg.pose.position.z = float(xyz[2])
            if yaw_rad is not None:
                # explicit yaw (HOME): quaternion from the bridge's own
                # euler convention (w = cos(t/2), z = sin(t/2))
                msg.pose.orientation.w, msg.pose.orientation.z = \
                    quat_from_yaw(yaw_rad)
            elif self.ee_pose is not None:
                # latest pose quaternion verbatim: the gripper yaw
                # passes through unchanged (round-trips through the
                # bridge's yaw convention)
                msg.pose.orientation = self.ee_pose.pose.orientation
            else:
                msg.pose.orientation.w = 1.0  # no pose seen yet: yaw 0
            self._goal_pub.publish(msg)

        def _spin_wait(self, predicate, timeout):
            """Nested spin until predicate() or timeout. The service
            callback's reentrant group lets the detection timer and the
            feedback subscriptions keep running while this blocks."""
            deadline = time.time() + float(timeout)
            while time.time() < deadline and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.05)
                if predicate():
                    return True
            return False

        def _dist_to(self, xyz):
            p = self.fresh_ee_xyz()
            if p is None:
                return None
            return float(np.linalg.norm(p - np.asarray(xyz, dtype=float)))

        def wait_reach(self, xyz, timeout):
            cmd = np.asarray(xyz, dtype=float)
            tol = float(self.params['reach_tol'])
            reached = self._spin_wait(
                lambda: self._dist_to(cmd) is not None
                and self._dist_to(cmd) <= tol, timeout)
            err = self._dist_to(cmd)
            if reached:
                return True, err, f'reached, err {err:.1f} mm'
            if err is None:
                return False, None, 'no fresh arm/ee_pose (bridge down?)'
            return False, err, f'err {err:.1f} mm'

        def wait_settled(self, xyz, timeout):
            """Settle check for DESCEND/LIFT: N consecutive FRESH
            samples inside reach_tol AND < SETTLE_DELTA_MM apart. A
            stale pose gap resets the run (the samples must be
            consecutive AND fresh)."""
            cmd = np.asarray(xyz, dtype=float)
            tracker = SettleTracker(tol=float(self.params['reach_tol']))
            deadline = time.time() + float(timeout)
            note = 'no fresh arm/ee_pose (bridge down?)'
            while time.time() < deadline and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.05)
                p = self.fresh_ee_xyz()
                if p is None:
                    tracker.reset()
                    continue
                settled, err, note = tracker.feed(p, cmd)
                if settled:
                    return True, err, note
            p = self.fresh_ee_xyz()
            err = float(np.linalg.norm(p - cmd)) \
                if p is not None else None
            if err is None:
                return False, None, note
            return False, err, f'not settled: {note}'

        def _clamp_err(self, cmd):
            if CLAMP_JOINT not in self.joint_pos:
                return None
            return abs(float(self.joint_pos[CLAMP_JOINT]) - cmd)

        def wait_clamp(self, rad, timeout):
            cmd = float(rad)
            tol = float(self.params['clamp_tol'])
            reached = self._spin_wait(
                lambda: self._clamp_err(cmd) is not None
                and self._clamp_err(cmd) <= tol, timeout)
            angle, cnote = self.read_clamp()
            if reached:
                return True, angle, f'reached ({angle:.3f} rad)'
            if angle is None:
                return False, None, 'no arm/joint_states (bridge down?)'
            return False, angle, \
                f'clamp at {angle:.3f} rad, err {abs(angle - cmd):.3f} rad'

        def read_clamp(self):
            if CLAMP_JOINT in self.joint_pos:
                return float(self.joint_pos[CLAMP_JOINT]), ''
            return None, 'no arm/joint_states yet'

        def read_pressure(self):
            if self.gripper is None:
                return None, None, 'no /gripper_pressure_reading yet ' \
                                   '(cmd_vel_serial running?)'
            age = time.time() - self.gripper_time
            note = f'pressure stale {age:.1f}s' \
                if age > PRESSURE_MAX_AGE_S else ''
            return int(self.gripper.hinge), int(self.gripper.anvil), note

        # --- detection loop (timer; keeps running during the sequence) ---

        def _open_camera(self):
            try:
                cap = cv2.VideoCapture(self.camera_device)
                if not cap.isOpened():
                    cap.release()
                    self.get_logger().warn(
                        f'cannot open camera {self.camera_device} — '
                        f'retrying every {CAM_RETRY_EVERY_S:.0f}s; the arm '
                        'sequence still runs from the grid JSON + '
                        'handeye.yaml')
                    return
                cap.set(cv2.CAP_PROP_FOURCC,
                        cv2.VideoWriter_fourcc(*'MJPG'))
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_height)
                self.cap = cap
                self.get_logger().info(
                    f'camera {self.camera_device} open '
                    f'({self.camera_width}x{self.camera_height}), '
                    'warming up next')
            except cv2.error as exc:  # pragma: no cover
                self.get_logger().warn(f'camera open failed: {exc}')

        def _setup_undistort(self, w, h):
            cam = load_camera_matrix(self.camera_matrix_yaml)
            if cam is None:
                self.maps = None
                return
            k, dist, calib_size = cam
            if calib_size is not None and tuple(calib_size) != (w, h):
                self.get_logger().warn(
                    f'capture {w}x{h} != calibrated '
                    f'{calib_size[0]}x{calib_size[1]} — K scaled')
                k = scale_intrinsics(k, calib_size, (w, h))
            try:
                self.maps = build_undistort_maps(k, dist, (w, h))
                self.get_logger().info('undistort ON')
            except cv2.error as exc:  # pragma: no cover
                self.get_logger().warn(
                    f'cannot build undistort maps: {exc} — RAW frames')
                self.maps = None

        def _tick(self):
            if self.detector is None:
                if not self._no_detector_warned:
                    self._no_detector_warned = True
                    self.get_logger().warn(
                        'dt-apriltags not importable — grid detection '
                        'OFF; the arm sequence still runs (grid JSON + '
                        'handeye.yaml). Install: pip3 install '
                        "'dt-apriltags>=3.1.7'")
                return
            if self.cap is None:
                if time.time() - self._cam_attempt_time >= \
                        CAM_RETRY_EVERY_S:
                    self._cam_attempt_time = time.time()
                    self._open_camera()
                return
            if not self._warmed:
                frame = warm_up(self.cap)
                if frame is None:
                    self.get_logger().warn(
                        'no bright frame after 60 reads — camera problem '
                        f'({self.camera_device}); keep retrying',
                        throttle_duration_sec=5.0)
                    return
                self._warmed = True
                h, w = frame.shape[:2]
                self._setup_undistort(w, h)
                self.get_logger().info(f'camera live {w}x{h}')
            ok, frame = self.cap.read()
            if not ok:
                self.get_logger().warn(
                    'camera read failed — keep retrying',
                    throttle_duration_sec=5.0)
                return
            work = undistort_frame(frame, self.maps) \
                if self.maps is not None else frame
            gray = cv2.cvtColor(work, cv2.COLOR_BGR2GRAY)
            detections = self.detector.detect(gray,
                                              estimate_tag_pose=False)
            obj_pts, img_pts, n_tags = match_corners(
                self.grid_tags, detections,
                min_tags=self.params['min_tags'])
            hh = None
            degraded = False
            if obj_pts is not None:
                hh = fit_homography(obj_pts, img_pts)
            else:
                # corner match failed but enough tags are visible:
                # degraded homography from tag centers only
                hh, n = center_homography(
                    self.grid_tags, detections,
                    min_tags=self.params['min_tags'])
                if hh is not None:
                    degraded = True
            if hh is not None and self.target_in is not None:
                self.target_px = apply_homography(
                    hh, np.array([self.target_in], dtype=float))[0]
                self.last_tags = n_tags
                self.detect_time = time.time()
                self._degraded = degraded
            else:
                self.last_tags = n_tags
            self._print_status()

        def _print_status(self):
            now = time.time()
            if now - self._last_status < STATUS_EVERY_S:
                return
            self._last_status = now
            dist_txt = 'n/a'
            pose = self.fresh_ee_xyz()
            if pose is not None and self.target_arm is not None:
                dist_txt = \
                    f'{np.linalg.norm(pose - self.target_arm):.0f} mm'
            if self.target_px is not None:
                extra = ' (center-only homography)' if self._degraded \
                    else ''
                self.get_logger().info(
                    f'tags: {self.last_tags}, target px '
                    f'({self.target_px[0]:.0f}, '
                    f'{self.target_px[1]:.0f}), dist-to-target '
                    f'{dist_txt}{extra}')
            elif self.last_tags:
                self.get_logger().info(
                    f'tags: {self.last_tags} (< {self.params["min_tags"]} '
                    '— target px unknown; some occlusion is expected '
                    'once the object sits on the grid)')
            else:
                self.get_logger().info(
                    'no grid tags in view yet (tag36h11 IDs 18-37)')

        # --- trigger service ---

        def _start_cb(self, req, resp):
            if self._busy:
                resp.success = False
                resp.message = 'sequence already running — one trigger ' \
                               'at a time'
                return resp
            if self.target_arm is None or self.normal is None:
                resp.success = False
                resp.message = ('no target: grid JSON or handeye.yaml '
                                'failed to load (see log)')
                return resp
            if self.fresh_ee_xyz() is None:
                # the bridge publishes arm/ee_pose only after its boot
                # drain + handshake; without a fresh pose, HOVER goals
                # would be dropped and the arm could sweep from its
                # boot pose straight to the grasp point.
                resp.success = False
                resp.message = ('arm bridge not ready — no fresh '
                                'arm/ee_pose yet; wait for the bridge '
                                'READY log, then trigger again')
                return resp
            self._busy = True
            rows = []
            try:
                if self.last_tags < self.params['min_tags']:
                    self.get_logger().warn(
                        f'grid NOT verified in the camera (last '
                        f'detection: {self.last_tags} tags) — moving to '
                        'the JSON/handeye target anyway (board must be in '
                        'the calibration position)')
                elif self.detect_time is None or \
                        time.time() - self.detect_time > \
                        DETECT_MAX_AGE_S:
                    age = 'never' if self.detect_time is None else \
                        f'{time.time() - self.detect_time:.0f}s ago'
                    self.get_logger().warn(
                        f'grid verification stale (last good detection '
                        f'{age}) — moving to the JSON/handeye target '
                        'anyway (camera is verification-only)')
                self.get_logger().warn(
                    'SAFETY: autonomous arm motion + grasping — keep '
                    'clear of the arm and the board. Torque stays '
                    'locked; an in-flight T:1041 move has no cancel path '
                    '(completes in ~2-3 s).')
                rows = run_sequence(self, self.target_arm, self.normal,
                                    self.params, self.get_logger())
                resp.success = True
                resp.message = (f'done: {len(rows)} stages logged — '
                                f'CSV in '
                                f'{Path(self.output_dir).expanduser()}')
            except Exception as exc:  # pragma: no cover — defensive
                resp.success = False
                resp.message = f'sequence aborted: {exc}'
                self.get_logger().warn(
                    f'sequence aborted: {exc!r} — partial rows logged')
            finally:
                self._busy = False
                if rows:
                    self._finish_logging(rows)
            return resp

        def _finish_logging(self, rows):
            outdir = Path(self.output_dir).expanduser()
            self._last_csv = None
            try:
                outdir.mkdir(parents=True, exist_ok=True)
                name = 'grasp_log_' + datetime.now().strftime(
                    '%Y%m%d_%H%M%S') + '.csv'
                self._last_csv = str(outdir / name)
                write_log_csv(self._last_csv, rows)
                self.get_logger().info(f'wrote {self._last_csv}')
            except OSError as exc:  # pragma: no cover
                self.get_logger().warn(
                    f'cannot write CSV to {outdir}: {exc}')
            self.get_logger().info('\n' + summarize(rows))


TERMINATION_SIGNALS = (
    (signal.SIGINT, 'SIGINT'),
    (signal.SIGTERM, 'SIGTERM'),
    (signal.SIGHUP, 'SIGHUP'),
)


def _make_signal_handler(name):  # pragma: no cover — runs on the Orin
    # handeye_capture.py's pattern (c383b35): rclpy's default SIGINT
    # handler shuts the context down before the service callback's
    # finally can write the partial CSV; raise KeyboardInterrupt so the
    # finally runs while rclpy is still alive. Registered for SIGINT,
    # SIGTERM and SIGHUP so a kill / terminal close takes the same
    # path. (SIGKILL cannot be caught.)
    def handler(signum, frame):
        print(f'{name} received — exiting (partial CSV will be '
              'written)', file=sys.stderr)
        raise KeyboardInterrupt
    return handler


def install_signal_handlers():  # pragma: no cover — runs on the Orin
    for sig, name in TERMINATION_SIGNALS:
        signal.signal(sig, _make_signal_handler(name))


def main(args=None):
    ok, msg = acquire_lock()
    if not ok:
        print(f'ERROR: {msg}', file=sys.stderr)
        return 2
    install_signal_handlers()
    rclpy.init(args=args)
    node = None
    try:
        node = GraspDemoNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        release_lock()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
