#!/usr/bin/env python3
"""T004 — hand-eye touch calibration capture (Orin, ROS2 + C270).

The user hand-guides the arm tip to 12 grid targets while a detection
window shows them as large red dots. SPACE records the current
arm/ee_pose (bridge-published RoArm FK); after 12 points the script
writes pairs.json, runs the Procrustes solver and writes handeye.yaml.

Requires the arm bridge (arm/ee_pose, arm/joint_states, arm/torque).
On startup torque is released (enable=false); torque is ALWAYS
re-locked on every exit path (try/finally). If the torque service is
unavailable the script warns loudly and continues RECORD-ONLY — the
user must release torque by hand.

Camera: C270 dark-start warm-up per T003 (60 reads, discard mean<5).
Undistort: the T003 camera_matrix.yaml (--camera-matrix) is applied
BEFORE detection and homography; a missing file warns and falls back
to raw frames (state shown on the control window).

Two windows:
  "detections"  live dt-apriltags overlay (quads + IDs) plus the 12
                target dots (filled red, radius ~20 px, thin white
                ring). Recorded targets are drawn green, the current
                target gets a yellow ring. Dots need >= 4 grid tags
                matched (homography grid-inches -> pixels); otherwise
                "move board into view" is shown.
  "control"     counter, current target label, arm pose fresh/stale
                indicator, undistort on/off, instructions.

Keys: SPACE record (fresh pose required, < 1.0 s), R re-record last,
Q/ESC quit (quits before 12 points exit 1; >= 3 recorded points still
write a partial pairs.json).

Exit codes:
  0  all 12 points recorded, pairs.json + handeye.yaml written
  1  quit / ended with fewer than 12 points
  2  camera, display, args, detector or ROS import problem

Core functions below the constants are pure numpy/cv2: no rclpy, no
serial — importable on the Mac (selftest.py). The ROS node is defined
only when the ROS interfaces import. This file never opens serial; all
arm I/O goes through the bridge's topics/services.
"""

import argparse
import json
import os
import signal
import sys
import time
from pathlib import Path

import cv2
import numpy as np

from solve_handeye import (arm_grid_arrays, dump_yaml, handeye_data,
                           print_solution, solve_rigid)

try:
    from dt_apriltags import Detector
except ImportError:  # pragma: no cover — the Orin has it
    Detector = None

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import JointState
    from sentinel_msgs.srv import SetTorque
    HAVE_ROS = True
except ImportError:  # pragma: no cover — Mac selftest has no ROS
    HAVE_ROS = False

DEFAULT_CAM = '/dev/sentinel_webcam'
DEFAULT_OUTDIR = '/home/sentinel1/dean/hand_eye'
DEFAULT_CAMERA_MATRIX = ('/home/sentinel1/dean/camera_calibration/'
                         'camera_matrix.yaml')
DEFAULT_GRID = 'grid_5x4_tag36h11_ids18-37.json'  # next to this script
DARK_THRESHOLD = 5.0
QUIET_ZONE_FRACTION = 0.8   # T002: detected corners are the black
                            # border, 8/10 of the printed tag side
MIN_TAGS_FOR_DOTS = 4
POSE_MAX_AGE_S = 1.0        # SPACE refuses older arm/ee_pose
N_TARGETS = 12


# ---------------------------------------------------------------------
# Core: grid, targets, homography, undistort — no ROS, no camera.
# ---------------------------------------------------------------------

def load_grid(json_path):
    """Return {tag_id: (cx_in, cy_in, side_in)} from the grid JSON."""
    data = json.loads(Path(json_path).read_text())
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


def _snap(v):
    """Kill JSON float noise near zero (outer corner must be exactly 0)."""
    return 0.0 if abs(v) < 1e-4 else round(v, 6)


def _extreme_corner(grid_tags, score):
    """(tag_id, corner, (x, y)) of the printed corner minimizing score."""
    best, best_score = None, None
    for tid, (cx, cy, side) in grid_tags.items():
        h = side / 2.0  # printed corner (quiet zone included)
        for corner, (px, py) in zip(('TL', 'TR', 'BR', 'BL'),
                                    ((cx - h, cy - h), (cx + h, cy - h),
                                     (cx + h, cy + h), (cx - h, cy + h))):
            s = score(px, py)
            if best_score is None or s < best_score:
                best, best_score = (tid, corner, (px, py)), s
    return best


# 8 interior tag corners spread across the central region of the board
# (grid frame: x right, y down; board is 7.4667 x 9.3333 in). Corners
# cover both sides and the middle: x spans 2.05-7.28 in, y spans
# 2.05-7.28 in. Corner names are in grid frame.
INTERIOR_CORNERS = [
    (23, 'TL'), (25, 'TR'), (31, 'BL'), (33, 'BR'),
    (27, 'TL'), (28, 'TR'), (27, 'BL'), (29, 'BR'),
]


def build_targets(grid_path):
    """The 12 touch targets + the parsed grid tags.

    Returns (targets, grid_tags). Each target dict:
      {'target': name, 'label': display text, 'tag_id': int|None,
       'corner': 'TL'|'TR'|'BL'|'BR', 'grid': [x_in, y_in]}
    The 4 outer corners are the grid's outermost printed corners
    (tag 0's printed TL corner is the origin, i.e. (0, 0) — the JSON
    side includes the quiet zone). The 8 interior corners are the
    black-border (0.8 x side) corners of INTERIOR_CORNERS — the same
    corners the detector reports, so the projected dots land on the
    visual tag corners.
    """
    grid_tags = load_grid(grid_path)
    (tid_tl, _, (x0, y0)) = _extreme_corner(grid_tags, lambda x, y: (x, y))
    (tid_tr, _, (x1, y0t)) = _extreme_corner(grid_tags,
                                             lambda x, y: (-x, y))
    (tid_br, _, (x1b, y1)) = _extreme_corner(grid_tags,
                                             lambda x, y: (-x, -y))
    (tid_bl, _, (x0b, y1b)) = _extreme_corner(grid_tags,
                                              lambda x, y: (x, -y))
    outer = [
        ('outer_TL', tid_tl, 'TL', (x0, y0)),
        ('outer_TR', tid_tr, 'TR', (x1, y0t)),
        ('outer_BR', tid_br, 'BR', (x1b, y1)),
        ('outer_BL', tid_bl, 'BL', (x0b, y1b)),
    ]
    targets = []
    for name, tid, corner, (px, py) in outer:
        targets.append({'target': name,
                        'label': f'outer {corner} corner',
                        'tag_id': int(tid), 'corner': corner,
                        'grid': [_snap(px), _snap(py)]})
    for tid, corner in INTERIOR_CORNERS:
        cx, cy, side = grid_tags[tid]
        px, py = dict(zip(('TL', 'TR', 'BR', 'BL'),
                          tag_object_corners(cx, cy, side)))[corner]
        targets.append({'target': f'interior_tag{tid}_{corner}',
                        'label': f'interior tag {tid} {corner} corner',
                        'tag_id': int(tid), 'corner': corner,
                        'grid': [round(px, 6), round(py, 6)]})
    assert len(targets) == N_TARGETS
    return targets, grid_tags


def detection_fields(d):
    """(tag_id, corners (4,2), center (2,)) in a detector-agnostic shape."""
    tid = int(d.tag_id)
    corners = np.asarray(d.corners, dtype=float).reshape(-1, 2)
    center = np.asarray(d.center, dtype=float).ravel()[:2]
    return tid, corners, center


def match_corners(grid_tags, detections, min_tags=MIN_TAGS_FOR_DOTS):
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


def center_homography(grid_tags, detections, min_tags=MIN_TAGS_FOR_DOTS):
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
    """Load (camera_matrix, dist_coeffs) from a T003 camera_matrix.yaml.

    Returns (K (3,3), dist (k,)) or None when the file is missing,
    unreadable, or lacks the keys — with a WARN on stderr (the caller
    then continues with raw frames)."""
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
        return k, dist
    except (OSError, ValueError, ImportError, TypeError) as exc:
        print(f'WARN: cannot read {p}: {exc} — continuing with RAW '
              'frames (undistort off)', file=sys.stderr)
        return None


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


def should_detect(frame_idx, detect_every):
    """True when the loop runs detection for this frame index."""
    return frame_idx % detect_every == 0


# ---------------------------------------------------------------------
# Drawing
# ---------------------------------------------------------------------

def draw_overlay(view, detections):
    """Tag quads + ID text + center dot (T003 style)."""
    out = view.copy()
    for d in detections:
        corners = np.asarray(d.corners, dtype=float).reshape(-1, 2)
        pts = np.round(corners).astype(int).reshape(-1, 1, 2)
        cv2.polylines(out, [pts], True, (0, 200, 0), 2)
        cx, cy = np.round(np.asarray(d.center, dtype=float).ravel()[:2])
        cx, cy = int(cx), int(cy)
        cv2.circle(out, (cx, cy), 3, (0, 0, 255), -1)
        cv2.putText(out, str(int(d.tag_id)), (cx + 8, cy - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 220, 0), 2)
    return out


def draw_targets(view, px_targets, n_recorded, current):
    """The 12 targets as filled red dots (r=20) with a thin white ring.

    Recorded targets are drawn green over the red; the current target
    gets a yellow highlight ring."""
    for i, (x, y) in enumerate(px_targets):
        p = (int(round(x)), int(round(y)))
        cv2.circle(view, p, 20, (0, 0, 255), -1)
        cv2.circle(view, p, 20, (255, 255, 255), 2)
        if i < n_recorded:
            cv2.circle(view, p, 12, (0, 200, 0), -1)
        if i == current:
            cv2.circle(view, p, 26, (0, 255, 255), 3)
    return view


def draw_control(view, n_recorded, targets, node, undistort_mode,
                 torque_warning):
    """Counter + target label + freshness + undistort + instructions."""
    out = view.copy()
    lines = [f'recorded {n_recorded}/12']
    idx = min(n_recorded, len(targets) - 1)
    lines.append(f'target {idx + 1}/12: {targets[idx]["label"]}')
    ee_age = node.ee_age() if node is not None else None
    if ee_age is None:
        lines.append('arm pose: none yet (bridge running?)')
    elif ee_age <= POSE_MAX_AGE_S:
        lines.append(f'arm pose: fresh ({ee_age:.1f}s)')
    else:
        lines.append(f'arm pose: STALE ({ee_age:.1f}s) — SPACE refused')
    js_age = node.joint_age() if node is not None else None
    if js_age is None:
        lines.append('joint states: none yet')
    elif js_age <= POSE_MAX_AGE_S:
        lines.append(f'joint states: fresh ({js_age:.1f}s)')
    else:
        lines.append(f'joint states: STALE ({js_age:.1f}s)')
    lines.append(f'undistort: {undistort_mode}')
    if torque_warning:
        lines.append('TORQUE SERVICE UNAVAILABLE — release arm by hand')
    lines.append('SPACE record, R re-record last, Q quit')
    y = 26
    for s in lines:
        cv2.putText(out, s, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (0, 220, 255), 2)
        y += 30
    return out


# ---------------------------------------------------------------------
# ROS node (defined only when the ROS interfaces import)
# ---------------------------------------------------------------------

if HAVE_ROS:  # pragma: no cover — runs on the Orin with ROS2

    class HandEyeCaptureNode(Node):

        def __init__(self):
            super().__init__('handeye_capture')
            self.ee_pose = None      # latest PoseStamped
            self.ee_time = None      # time.time() at receipt
            self.joint_time = None
            self.joint_pos = {}
            self._ee_sub = self.create_subscription(
                PoseStamped, 'arm/ee_pose', self._ee_cb, 10)
            self._js_sub = self.create_subscription(
                JointState, 'arm/joint_states', self._js_cb, 10)
            self._torque_cli = self.create_client(SetTorque, 'arm/torque')

        def _ee_cb(self, msg):
            self.ee_pose = msg
            self.ee_time = time.time()

        def _js_cb(self, msg):
            self.joint_time = time.time()
            self.joint_pos = dict(zip(msg.name, msg.position))

        def ee_age(self):
            return None if self.ee_time is None else time.time() - self.ee_time

        def joint_age(self):
            return None if self.joint_time is None \
                else time.time() - self.joint_time

        def torque(self, enable):
            """Call arm/torque; returns (ok, message). Safe to call from
            the main loop (blocks on spin_once until the reply or 3 s)."""
            if not self._torque_cli.wait_for_service(timeout_sec=3.0):
                return False, 'arm/torque service unavailable'
            req = SetTorque.Request()
            req.enable = enable
            fut = self._torque_cli.call_async(req)
            deadline = time.time() + 3.0
            while rclpy.ok() and not fut.done() and time.time() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
            if not fut.done():
                return False, 'torque call timed out'
            resp = fut.result()
            if resp is None or not resp.success:
                return False, 'bridge refused torque call (arm not ready?)'
            return True, 'ok'


def snapshot_pose(node):
    """Latest fresh arm/ee_pose as (x, y, z) mm, or (None, reason).

    Single latest pose, no averaging: the user records while the arm is
    stationary, and averaging would blend motion if the arm sags."""
    age = node.ee_age()
    if node.ee_pose is None:
        return None, 'no arm/ee_pose received yet (bridge running?)'
    if age > POSE_MAX_AGE_S:
        return None, f'arm pose stale ({age:.1f}s > {POSE_MAX_AGE_S:.1f}s)'
    p = node.ee_pose.pose.position
    return (float(p.x), float(p.y), float(p.z)), None


def _sigint_handler(signum, frame):  # pragma: no cover
    # rclpy's default SIGINT handler shuts the context down before the
    # finally can re-lock torque; raise instead so the finally runs
    # while rclpy is still alive.
    raise KeyboardInterrupt


def write_pairs(pairs, outdir, path_name='pairs.json'):
    """Write pairs.json (unit_grid/unit_arm + points). Returns path."""
    outdir.mkdir(parents=True, exist_ok=True)
    path = outdir / path_name
    path.write_text(json.dumps(pairs, indent=2) + '\n')
    return path


def finish_run(recorded, outdir):
    """Write pairs.json, solve, write handeye.yaml, print the result."""
    pairs = {'unit_grid': 'in', 'unit_arm': 'mm',
             'points': recorded}
    pairs_path = write_pairs(pairs, outdir)
    print(f'wrote {pairs_path}')
    arm, grid, _ = arm_grid_arrays(recorded, 'in')
    r, t, rms, residuals = solve_rigid(arm, grid)
    print_solution(recorded, r, t, rms, residuals)
    yaml_path = outdir / 'handeye.yaml'
    writer = dump_yaml(yaml_path, handeye_data(recorded, r, t, rms,
                                               residuals))
    print(f'wrote {yaml_path} ({writer})')


def parse_args(argv):
    ap = argparse.ArgumentParser(
        description='T004 hand-eye touch calibration capture (Orin).')
    ap.add_argument('--cam', default=DEFAULT_CAM,
                    help='camera device (default %(default)s)')
    ap.add_argument('--width', type=int, default=1280)
    ap.add_argument('--height', type=int, default=720)
    ap.add_argument('--grid', default=None,
                    help='grid JSON (default: next to this script)')
    ap.add_argument('--outdir', default=DEFAULT_OUTDIR,
                    help='output dir for pairs.json/handeye.yaml '
                         '(default %(default)s)')
    ap.add_argument('--camera-matrix', default=DEFAULT_CAMERA_MATRIX,
                    help='T003 camera_matrix.yaml (default %(default)s)')
    return ap.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)

    if not HAVE_ROS:
        print('ERROR: rclpy or the ROS interfaces are not importable — '
              'source the ROS workspace first '
              '(source ~/ros2_ws/install/setup.bash)', file=sys.stderr)
        return 2
    if not os.environ.get('DISPLAY'):
        print('ERROR: no DISPLAY — the two capture windows need a '
              'graphical session on the Orin (run at the robot\'s '
              'desktop, or ssh with X forwarding).', file=sys.stderr)
        return 2
    if Detector is None:
        print('ERROR: dt-apriltags is not installed on the Orin '
              '(run: pip3 install -r requirements.txt)', file=sys.stderr)
        return 2

    grid_path = Path(args.grid) if args.grid else \
        Path(__file__).resolve().parent / DEFAULT_GRID
    if not grid_path.is_file():
        print(f'ERROR: grid JSON not found: {grid_path}', file=sys.stderr)
        return 2
    try:
        targets, grid_tags = build_targets(grid_path)
    except (OSError, ValueError, KeyError, json.JSONDecodeError) as exc:
        print(f'ERROR: cannot load grid {grid_path}: {exc}',
              file=sys.stderr)
        return 2
    print(f'grid: {grid_path} ({len(grid_tags)} tags, '
          f'{len(targets)} targets)')

    cap = cv2.VideoCapture(args.cam)
    if not cap.isOpened():
        print(f'ERROR: cannot open camera {args.cam}', file=sys.stderr)
        return 2
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    print('warming up camera (C270 auto-exposure ramp) ...')
    frame = warm_up(cap)
    if frame is None:
        print('ERROR: no bright frame after 60 reads — camera problem '
              f'({args.cam}); check the cable and the udev symlink.',
              file=sys.stderr)
        cap.release()
        return 2
    print('live')
    height, width = frame.shape[:2]
    print(f'capture {width}x{height}')

    cam = load_camera_matrix(args.camera_matrix)
    maps = None
    undistort_mode = 'RAW'
    if cam is not None:
        k, dist = cam
        try:
            maps = build_undistort_maps(k, dist, (width, height))
        except cv2.error as exc:
            print(f'WARN: cannot build undistort maps: {exc} — '
                  'continuing with RAW frames', file=sys.stderr)
            maps = None
        if maps is not None:
            undistort_mode = 'ON'
            print(f'undistort ON ({args.camera_matrix})')
    if maps is None:
        print('undistort OFF — RAW frames (see warning above)')

    try:
        cv2.namedWindow('detections', cv2.WINDOW_NORMAL)
        cv2.namedWindow('control', cv2.WINDOW_NORMAL)
    except cv2.error as exc:
        print(f'ERROR: cannot create GUI windows ({exc}) — the two '
              'windows need a display session on the Orin.',
              file=sys.stderr)
        cap.release()
        return 2

    # --- ROS startup (only after the camera is live) ---
    rclpy.init()
    signal.signal(signal.SIGINT, _sigint_handler)
    node = HandEyeCaptureNode()
    torque_warning = None
    detector = Detector(families='tag36h11', quad_decimate=1.0)
    recorded = []
    current = 0
    exit_code = 1
    detections = []
    detect_every = 1
    frame_idx = 0
    px_targets = None

    try:
        ok, msg = node.torque(False)
        if not ok:
            torque_warning = msg
            print(f'WARN: torque release FAILED: {msg} — continuing '
                  'RECORD-ONLY (release torque by hand)', file=sys.stderr)
        else:
            print('torque released — arm is limp, hand-guide the tip '
                  'to each red dot')

        while rclpy.ok():
            ok_read, frame = cap.read()
            if not ok_read:
                continue
            frame_idx += 1
            work = undistort_frame(frame, maps) if maps is not None else frame

            if should_detect(frame_idx, detect_every):
                gray = cv2.cvtColor(work, cv2.COLOR_BGR2GRAY)
                detections = detector.detect(gray, estimate_tag_pose=False)
                obj_pts, img_pts, n_tags = match_corners(
                    grid_tags, detections)
                if obj_pts is not None:
                    hh = fit_homography(obj_pts, img_pts)
                else:
                    hh = center_homography(grid_tags, detections)[0]
                if hh is not None:
                    pts = np.array([t['grid'] for t in targets],
                                   dtype=float)
                    px_targets = apply_homography(hh, pts)
                else:
                    px_targets = None

            overlay = draw_overlay(work, detections)
            if px_targets is not None:
                draw_targets(overlay, px_targets, len(recorded), current)
            else:
                cv2.putText(overlay, 'move board into view '
                            f'({MIN_TAGS_FOR_DOTS}+ grid tags needed)',
                            (12, 36), cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                            (0, 0, 255), 2)

            control = draw_control(frame, len(recorded), targets, node,
                                   undistort_mode, torque_warning)
            cv2.imshow('detections', overlay)
            cv2.imshow('control', control)
            key = cv2.waitKey(1) & 0xFF
            rclpy.spin_once(node, timeout_sec=0.01)

            if key in (ord('q'), ord('Q'), 27):
                print(f'quit: {len(recorded)}/12 recorded')
                break
            if key == 32:  # SPACE: record
                xyz, err = snapshot_pose(node)
                if err:
                    print(f'refused: {err}')
                else:
                    t = targets[current]
                    recorded.append({
                        'target': t['target'],
                        'tag_id': t['tag_id'],
                        'corner': t['corner'],
                        'grid': t['grid'],
                        'arm_xyz_mm': [round(v, 3) for v in xyz],
                    })
                    print(f'recorded {len(recorded)}/12: {t["label"]} '
                          f'arm=({xyz[0]:.1f}, {xyz[1]:.1f}, '
                          f'{xyz[2]:.1f}) mm')
                    current += 1
                    if current >= N_TARGETS:
                        finish_run(recorded, Path(args.outdir))
                        exit_code = 0
                        break
            elif key in (ord('r'), ord('R')):  # R: re-record last
                if recorded:
                    recorded.pop()
                    current = len(recorded)
                    print(f're-recording target {current + 1}/12: '
                          f'{targets[current]["label"]}')
                else:
                    print('nothing to re-record yet')

        if exit_code != 0 and len(recorded) >= 3:
            # Keep whatever the user recorded: the solver accepts >= 3.
            partial = write_pairs({'unit_grid': 'in', 'unit_arm': 'mm',
                                   'points': recorded},
                                  Path(args.outdir), 'pairs_partial.json')
            print(f'partial pairs.json written: {partial} '
                  f'({len(recorded)} points)')
        if exit_code != 0:
            print(f'exiting: {len(recorded)}/12 points '
                  '(need all 12 for a full solve)')
    except KeyboardInterrupt:
        print('interrupted')
    finally:
        # Safety: ALWAYS re-lock torque, on every exit path.
        ok, msg = node.torque(True)
        if ok:
            print('torque re-locked')
        else:
            print(f'WARN: torque re-lock FAILED: {msg} — re-lock manually: '
                  'ros2 service call arm/torque sentinel_msgs/srv/SetTorque '
                  '"{enable: true}"', file=sys.stderr)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        cap.release()
        cv2.destroyAllWindows()
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
