#!/usr/bin/env python3
"""T006 — hand-eye verification: visit the 12 grid targets (Orin, ROS2).

Drives the arm in END-EFFECTOR space through the bridge's arm/ee_goals
topic (geometry_msgs/PoseStamped -> T:1041 XYZ direct, mm). For each of
the 12 T004 grid targets (board plane, grid inches, z=0):

    arm_xyz = R^T @ (grid_mm - t)                 (handeye.yaml)
    arm_xyz += z_lift * n                         (n = grid normal in
                                                   arm frame)

with n signed so +z_lift raises the tip AWAY from the board plane
(default 10 mm; --z-lift 0 = exact on-plane). One move per target:
publish arm/ee_goals, wait until the bridge's arm/ee_pose is within
--reach-tol of the commanded point (--timeout), hold --hold seconds,
then the next target. Per-move spd is computed as
clamp(dist / --move-time, --spd-min, --spd-max) and printed — the
ACTUAL T:1041 spd/acc come from the bridge's ee_spd/ee_acc node params
(defaults 100/40); see the spd-unit note below.

The orientation in each goal is the latest fresh arm/ee_pose
quaternion, passed through unchanged, so the gripper yaw stays where
it is; the bridge converts it back to t with the same yaw convention
it publishes (round-trip consistent).

Safety:
  * NEVER releases torque, NEVER opens serial — the only arm command
    this script sends is arm/ee_goals publications.
  * Ctrl+C / SIGTERM / SIGHUP / Q (then Enter) stop the sequence
    immediately: no new goals are published. T:1041 is ONE autonomous
    command with no cancel path, so an in-flight move COMPLETES first
    (~2-3 s at the default bridge spd) and only then does the arm
    hold — keep clear for those seconds. Torque stays locked (every
    T:1041 re-locks it).
  * The FIRST unreached target STOPS the whole sequence (documented
    decision: no flailing — an unreached target usually means a bad
    calibration or a bad command, and continuing could drive the arm
    into something). Warn + exit 1.

Exit codes:
  0  all 12 targets reached
  1  stopped (Ctrl+C/Q) or a target not reached in time
  2  args / missing or bad handeye.yaml or grid / no ROS environment

T:1041 spd units ASSUMPTION (unverified on hardware): the repo docs
(docs/arm_teach_repeat_plan.md) do not document T:1041 spd/acc units.
We assume the same unit scale as the bridge's existing T:101 moves and
its T:1041 home move (spd 200 / acc 40), i.e. the spd field takes
values of the same magnitude the computed mm/s values have. The
bridge's ee_spd/ee_acc params are the actual knobs (launch args).
"""

import argparse
import os
import select
import signal
import sys
import time
from pathlib import Path

import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped
    HAVE_ROS = True
except ImportError:  # pragma: no cover — Mac selftest has no ROS
    HAVE_ROS = False

DEFAULT_HANDEYE = '~/dean/hand_eye/handeye.yaml'
DEFAULT_GRID_NAME = 'grid_5x4_tag36h11_ids18-37.json'
POSE_MAX_AGE_S = 1.0     # fresh = arm/ee_pose younger than this
N_TARGETS = 12
IN_TO_MM = 25.4
READY_STATUS_EVERY_S = 5.0
NORMAL_Z_WARN = 0.2      # |normal.arm_z| below this: sign check shaky
LOCK_PATH = '/tmp/test_handeye.lock'


# ---------------------------------------------------------------------
# Core: handeye math, targets, spd — no ROS. Importable on the Mac.
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

    How we know the sign: the grid frame is 2D on the board (x right,
    y down, z = 0 in the plane — T004). The grid +z direction in arm
    space is n0 = R^T @ [0,0,1]. Every calibration point lies in the
    board plane, so the Procrustes solve pins n0 only up to SIGN — the
    yaml's R may have grid +z pointing THROUGH the board instead of
    away from it. The physical setup disambiguates: the arm reaches
    the board from above (arm frame x forward / y left / z UP per the
    bridge's T:1041 convention), so "away from the board" must have a
    positive arm-frame z component. If n0 points downward (n0[2] < 0)
    we flip it. When |n0[2]| is small (board not below the arm, e.g.
    mounted vertically) this check is unreliable — we warn instead of
    guessing silently."""
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


def target_arm_xyz(grid_mm, r, t, z_lift, normal):
    """Board-plane grid point (mm, z=0) -> commanded arm XYZ (mm),
    lifted by z_lift mm along `normal` (lift_direction(r)[0] — the
    grid normal in the arm frame, away from the board)."""
    arm = grid_to_arm(grid_mm, r, t)
    if z_lift:
        arm = arm + float(z_lift) * np.asarray(normal, dtype=float)
    return arm


def compute_spd(dist_mm, move_time, spd_min, spd_max):
    """T:1041 spd for a dist_mm move planned to take move_time s,
    clamped to [spd_min, spd_max] — i.e. mm/s in the assumed spd scale
    (see the module-docstring spd-unit note)."""
    return min(float(spd_max),
               max(float(spd_min), float(dist_mm) / float(move_time)))


def load_targets(grid_path):
    """The 12 T004 targets from the grid JSON (order = capture order):
    [{'target': name, 'grid': [x_in, y_in]}, ...]. Reuses
    handeye_capture.build_targets so the visit list matches the
    calibration list exactly."""
    try:
        from handeye_capture import build_targets
    except ImportError as exc:  # pragma: no cover
        raise ValueError('cannot import handeye_capture.py (needs '
                         f'numpy + cv2): {exc}') from exc
    targets, _ = build_targets(grid_path)
    return targets


def resolve_grid(args_grid, handeye_path):
    """Path of the grid JSON: explicit --grid, else next to the
    handeye.yaml, else next to this script. None when absent."""
    candidates = []
    if args_grid:
        candidates.append(Path(args_grid))
    candidates.append(Path(handeye_path).with_name(DEFAULT_GRID_NAME))
    candidates.append(Path(__file__).resolve().parent / DEFAULT_GRID_NAME)
    for p in candidates:
        if p.is_file():
            return p
    return None


# ---------------------------------------------------------------------
# ROS node (defined only when the ROS interfaces import)
# ---------------------------------------------------------------------

if HAVE_ROS:  # pragma: no cover — runs on the Orin with ROS2

    class TestHandEyeNode(Node):

        def __init__(self):
            super().__init__('test_handeye')
            self.ee_pose = None      # latest PoseStamped
            self.ee_time = None      # time.time() at receipt
            self._ee_sub = self.create_subscription(
                PoseStamped, 'arm/ee_pose', self._ee_cb, 10)
            self._goal_pub = self.create_publisher(
                PoseStamped, 'arm/ee_goals', 10)

        def _ee_cb(self, msg):
            self.ee_pose = msg
            self.ee_time = time.time()

        def ee_age(self):
            return None if self.ee_time is None \
                else time.time() - self.ee_time

        def fresh_pose(self):
            """Latest arm/ee_pose as (x, y, z) mm, or None when stale."""
            if self.ee_pose is None or self.ee_age() > POSE_MAX_AGE_S:
                return None
            p = self.ee_pose.pose.position
            return np.array([p.x, p.y, p.z], dtype=float)

        def latest_pose(self):
            """Latest arm/ee_pose (fresh or not), or None."""
            if self.ee_pose is None:
                return None
            p = self.ee_pose.pose.position
            return np.array([p.x, p.y, p.z], dtype=float)

        def publish_goal(self, xyz):
            """Publish an arm/ee_goals PoseStamped: position = xyz mm;
            orientation = the latest ee_pose quaternion verbatim, so
            the gripper yaw (t) passes through unchanged and round-
            trips exactly through the bridge's yaw convention."""
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'arm_base_link'
            msg.pose.position.x = float(xyz[0])
            msg.pose.position.y = float(xyz[1])
            msg.pose.position.z = float(xyz[2])
            if self.ee_pose is not None:
                msg.pose.orientation = self.ee_pose.pose.orientation
            else:  # no pose seen yet: identity (yaw 0)
                msg.pose.orientation.w = 1.0
            self._goal_pub.publish(msg)


# ---------------------------------------------------------------------
# Stop handling: signals and terminal Q (no torque work — this script
# never releases torque).
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
    """Single-instance guard (two concurrent runs would interleave
    goal streams on one arm). Returns (ok, message): refuses when
    another LIVE instance holds the lock; clears stale (dead-pid)
    locks; writes our pid into the file."""
    p = Path(path)
    if p.exists():
        old_pid = None
        try:
            old_pid = int(p.read_text().strip())
        except (OSError, ValueError):
            pass
        if old_pid is not None and pid_alive(old_pid):
            return False, (f'another test_handeye.py (pid {old_pid}) '
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


TERMINATION_SIGNALS = (
    (signal.SIGINT, 'SIGINT'),
    (signal.SIGTERM, 'SIGTERM'),
    (signal.SIGHUP, 'SIGHUP'),
)


def _make_signal_handler(name):  # pragma: no cover — runs on the Orin
    # handeye_capture's pattern: raise KeyboardInterrupt so the main
    # loop exits cleanly while rclpy is still alive. No torque re-lock
    # in the finally — torque was never released here. T:1041 has no
    # cancel path: an in-flight move COMPLETES (~2-3 s) before the arm
    # holds; the message below says so.
    def handler(signum, frame):
        print(f'{name} received — stopping the sequence; an in-flight '
              'move (if any) completes first (~2-3 s), then the arm '
              'holds. Torque stays locked.', file=sys.stderr)
        raise KeyboardInterrupt
    return handler


def install_signal_handlers():  # pragma: no cover — runs on the Orin
    for sig, name in TERMINATION_SIGNALS:
        signal.signal(sig, _make_signal_handler(name))


class StopRequested(Exception):
    """Q typed in the terminal — same handling as Ctrl+C."""


def stdin_quit():
    """True when the user typed q/Q (then Enter) on stdin. Non-blocking,
    safe on pipes/closed stdin (Ctrl+C always works)."""
    try:
        if select.select([sys.stdin], [], [], 0.0)[0]:
            line = sys.stdin.readline()
            return bool(line) and line.strip().lower() == 'q'
    except (OSError, ValueError):  # pragma: no cover
        pass
    return False


# ---------------------------------------------------------------------
# Main flow
# ---------------------------------------------------------------------

def parse_args(argv):
    ap = argparse.ArgumentParser(
        description='T006 hand-eye verification: visit the 12 grid '
                    'targets via arm/ee_goals (T:1041, end-effector '
                    'space).')
    ap.add_argument('--handeye', default=DEFAULT_HANDEYE,
                    help='handeye.yaml from solve_handeye.py '
                         '(default %(default)s)')
    ap.add_argument('--grid', default=None,
                    help='grid JSON (default: next to the handeye '
                         'yaml, else next to this script)')
    ap.add_argument('--z-lift', type=float, default=10.0,
                    help='lift above the board plane along the grid '
                         'normal, mm (0 = exact on-plane) '
                         '(default %(default)s)')
    ap.add_argument('--reach-tol', type=float, default=25.0,
                    help='reached = |ee_pose - commanded| below this, '
                         'mm (default %(default)s; the first '
                         'calibration\'s worst residual was 22 mm, so '
                         '25 mm is the sane first-run default)')
    ap.add_argument('--move-time', type=float, default=2.0,
                    help='planned seconds per move; computed spd = '
                         'clamp(dist/move-time, spd-min, spd-max) '
                         '(default %(default)s)')
    ap.add_argument('--hold', type=float, default=1.0,
                    help='hold at each target, s (default %(default)s)')
    ap.add_argument('--timeout', type=float, default=5.0,
                    help='per-target reach timeout, s '
                         '(default %(default)s)')
    ap.add_argument('--spd-min', type=float, default=50.0,
                    help='computed-spd floor (default %(default)s)')
    ap.add_argument('--spd-max', type=float, default=300.0,
                    help='computed-spd ceiling (default %(default)s)')
    return ap.parse_args(argv)


def wait_fresh_pose(node, status_every=READY_STATUS_EVERY_S):
    """Block until a fresh (< 1 s) arm/ee_pose arrives (the bridge's
    boot drain takes ~8-15 s). Raises StopRequested on Q."""
    last = 0.0
    while True:
        rclpy.spin_once(node, timeout_sec=0.05)
        if stdin_quit():
            raise StopRequested
        if node.fresh_pose() is not None:
            return
        now = time.time()
        if now - last >= status_every:
            print(f'waiting for a fresh arm/ee_pose (bridge boot '
                  f'drain ~8-15 s) ... Q or Ctrl+C to stop')
            last = now


def wait_reach(node, commanded, timeout, tol):
    """Spin until a fresh arm/ee_pose is within tol mm of `commanded`.
    Returns (reached, err, pose): the satisfying fresh pose (reached),
    or the latest pose at timeout (not reached; None if none ever
    arrived). Raises StopRequested on Q."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if stdin_quit():
            raise StopRequested
        pose = node.fresh_pose()
        if pose is not None:
            err = float(np.linalg.norm(pose - commanded))
            if err <= tol:
                return True, err, pose
    pose = node.latest_pose()
    err = float(np.linalg.norm(pose - commanded)) \
        if pose is not None else None
    return False, err, pose


def hold(seconds, node):
    """Hold the current pose for `seconds` with Q checks; raises
    StopRequested on Q."""
    deadline = time.time() + seconds
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if stdin_quit():
            raise StopRequested
        time.sleep(0.05)


def main(argv=None):
    args = parse_args(argv)

    if not HAVE_ROS:
        print('ERROR: rclpy or the ROS interfaces are not importable — '
              'source the ROS workspace first '
              '(source ~/ros2_ws/install/setup.bash)', file=sys.stderr)
        return 2

    handeye_path = Path(args.handeye).expanduser()
    try:
        r, t = load_handeye(handeye_path)
    except ValueError as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        return 2
    grid_path = resolve_grid(args.grid, handeye_path)
    if grid_path is None:
        print(f'ERROR: grid JSON {DEFAULT_GRID_NAME} not found (tried '
              'next to the handeye.yaml and next to this script) — '
              'pass --grid', file=sys.stderr)
        return 2
    try:
        targets = load_targets(grid_path)
    except (ValueError, OSError) as exc:
        print(f'ERROR: cannot load grid {grid_path}: {exc}',
              file=sys.stderr)
        return 2
    if len(targets) != N_TARGETS:
        print(f'ERROR: expected {N_TARGETS} targets in the grid, got '
              f'{len(targets)}', file=sys.stderr)
        return 2

    normal, flipped = lift_direction(r)
    print(f'handeye: {handeye_path}')
    print(f'grid:    {grid_path} ({len(targets)} targets)')
    print(f'lift:    {args.z_lift:.1f} mm along grid normal '
          f'({normal[0]:+.3f}, {normal[1]:+.3f}, {normal[2]:+.3f}) in '
          f'arm frame — '
          f'{("yaml grid-z sign FLIPPED (it pointed through the board)" if flipped else "sign as in the yaml R")}')
    print(f'reach:   tol {args.reach_tol:.1f} mm, timeout '
          f'{args.timeout:.1f} s, hold {args.hold:.1f} s, '
          f'move-time {args.move_time:.1f} s')
    if args.z_lift < 0:
        print('WARN: negative --z-lift pushes TOWARD the board — '
              're-check before running!', file=sys.stderr)
    print('WARN: if handeye_capture.py just ran, the arm may still be '
          'torque-released (limp) — support it before starting; the '
          'first T:1041 re-locks torque and moves it.', file=sys.stderr)
    print('WARN: the board must be in the same position as during the '
          'pairs.json capture (hand-eye calibration).', file=sys.stderr)
    print('SAFETY: autonomous arm motion — keep clear of the arm and '
          'board. Q or Ctrl+C stops the sequence; an in-flight move '
          '(if any) completes first (~2-3 s), then the arm holds. '
          'Torque stays locked.')

    # Acquired only after every args/env/file check above, so all the
    # exit-2 paths leave no lock behind; the try/finally below always
    # releases it. (A lock lost to a hard crash self-heals: the next
    # start sees a dead pid and clears it.)
    ok, msg = acquire_lock()
    if not ok:
        print(f'ERROR: {msg}', file=sys.stderr)
        return 2

    rclpy.init()
    install_signal_handlers()
    node = TestHandEyeNode()
    exit_code = 1
    achieved = []  # (name, commanded, actual, err) per reached target
    try:
        wait_fresh_pose(node)
        start = node.fresh_pose()
        print(f'bridge ready — arm at ({start[0]:.1f}, {start[1]:.1f}, '
              f'{start[2]:.1f}) mm')
        prev = start
        stopped_early = False
        for k, target in enumerate(targets):
            g = np.array([target['grid'][0], target['grid'][1], 0.0],
                         dtype=float) * IN_TO_MM
            cmd = target_arm_xyz(g, r, t, args.z_lift, normal)
            dist = float(np.linalg.norm(cmd - prev))
            spd = compute_spd(dist, args.move_time, args.spd_min,
                              args.spd_max)
            node.publish_goal(cmd)
            print(f'-> {k + 1}/{N_TARGETS} {target["target"]} '
                  f'({target["label"]}): cmd ({cmd[0]:.1f}, {cmd[1]:.1f}, '
                  f'{cmd[2]:.1f}) mm, dist {dist:.0f} mm, computed spd '
                  f'{spd:.0f} (actual T:1041 spd = bridge ee_spd '
                  f'param, default 100)')
            reached, err, actual = wait_reach(
                node, cmd, args.timeout, args.reach_tol)
            if not reached:
                err_txt = f'{err:.1f}' if err is not None else 'n/a'
                print(f'WARN: target {k + 1} ({target["target"]}) not '
                      f'reached in {args.timeout:.1f} s, err '
                      f'{err_txt} mm')
                if err is not None:
                    print(f'hint: retry with --reach-tol '
                          f'{err + 5.0:.0f} mm (this target missed by '
                          f'{err:.1f} mm)')
                else:
                    print('hint: no arm/ee_pose at all — check the '
                          'bridge (ros2 topic hz arm/ee_pose)')
                print('STOP: aborting the sequence after the first '
                      'unreached target (safety — no flailing). An '
                      'in-flight move (if any) completes, then the arm '
                      'holds its current pose, torque stays locked.')
                stopped_early = True
                break
            hold(args.hold, node)
            print(f'{k + 1}/{N_TARGETS}: cmd ({cmd[0]:.1f}, {cmd[1]:.1f}, '
                  f'{cmd[2]:.1f}) achieved ({actual[0]:.1f}, '
                  f'{actual[1]:.1f}, {actual[2]:.1f}) err {err:.1f} mm')
            achieved.append((target['target'], cmd, actual, err))
            prev = cmd
        if not stopped_early:
            errs = np.array([a[3] for a in achieved])
            rms = float(np.sqrt(np.mean(errs ** 2)))
            worst_i = int(np.argmax(errs))
            name, cmd_w, act_w, err_w = achieved[worst_i]
            print(f'RMS achieved-vs-commanded: {rms:.2f} mm over '
                  f'{len(achieved)} targets')
            print(f'max error: {name} {err_w:.1f} mm '
                  f'(cmd ({cmd_w[0]:.1f}, {cmd_w[1]:.1f}, {cmd_w[2]:.1f}) '
                  f'-> ({act_w[0]:.1f}, {act_w[1]:.1f}, {act_w[2]:.1f}))')
            exit_code = 0
    except (KeyboardInterrupt, StopRequested):
        print('stopped — no new goals; an in-flight move (if any) '
              'completes (~2-3 s), then the arm holds')
    finally:
        release_lock()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
