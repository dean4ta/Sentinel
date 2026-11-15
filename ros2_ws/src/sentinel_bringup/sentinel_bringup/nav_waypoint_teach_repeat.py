#!/usr/bin/env python3
"""T010 nav waypoint teach+repeat.

Teach: drive the robot by hand (gamepad teleop, launched with bringup) and
press SPACE to record the current /amcl_pose as a waypoint. Repeat: navigate
to each recorded waypoint via the NavigateToPose action (bt_navigator), with
per-waypoint retries.

Run:
  ros2 run sentinel_bringup nav_waypoint_teach_repeat --teach
  ros2 run sentinel_bringup nav_waypoint_teach_repeat --repeat <file.yaml>

Design notes
------------
* Waypoints are MAP-frame poses from /amcl_pose. Teach and repeat sessions
  must start from the SAME physical pose: bringup's initial_pose_pub sets a
  low-confidence map-origin initial pose (0, 0, 0) ~8 s after launch, so the
  robot is parked at the map origin before launching. Waypoints are only
  valid against that origin; any drift between sessions is a start-pose
  violation, not a waypoint-file bug.
* Playback is our own engine driving the NavigateToPose action. The stock
  nav2 waypoint_follower is not used: it stops on failure (stop_on_failure)
  and has no per-waypoint retries.
* Goal orientation: face the NEXT waypoint when one exists
  (yaw = atan2(next-current)); identity quaternion (yaw 0, i.e. facing map
  +x) for the last waypoint. Teach records position only, not heading, and
  the last waypoint has no next one to face, so identity is the deterministic
  choice; DWB converges to it after arrival. Facing the next waypoint
  pre-orients the robot along its next travel leg (lidar pointed along the
  route, no extra in-place turn at the waypoint).

YAML format (nav2 waypoint-follower style)
------------------------------------------
Waypoint files are YAML documents with a `waypoints:` list of mappings. We
write only name + position:

    waypoints:
    - name: wp1
      position:
        x: 1.0
        y: 2.0
        z: 0.0
    - name: wp2
      position:
        x: 3.0
        y: 4.0
        z: 0.0

The loader also accepts the extra per-waypoint fields the nav2 waypoint
follower format allows (max_linear_velocity, max_angular_velocity) plus any
other unknown keys: they are read and ignored, never required. `z` defaults
to 0.0 and a missing `name` defaults to wpN (1-based index). Files we save
never contain the velocity fields.

Everything above the 'ROS wiring' seam below is pure Python (no rclpy), so
the logic tests on the Mac without ROS.
"""

import math
import os
import select
import sys
import termios
import tty

import yaml

DEFAULT_WAYPOINT_DIR = '/home/sentinel1/dean/Sentinel/nav_waypoints'
DEFAULT_WAYPOINT_FILE = 'waypoints.yaml'

REACHED = 'reached'
SKIPPED = 'skipped'
RETRY = 'retry'

IDENTITY_QUATERNION = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}


class WaypointError(Exception):
    """Invalid waypoint data or waypoint file."""


class Waypoint:
    """One map-frame waypoint: a name and a position (x, y, z in meters).

    max_linear_velocity / max_angular_velocity are accepted when loading a
    nav2 waypoint-follower file (we never write them): kept as floats if
    present, None otherwise. Our playback ignores them.
    """

    def __init__(self, name, x, y, z=0.0,
                 max_linear_velocity=None, max_angular_velocity=None):
        if not isinstance(name, str) or not name.strip():
            raise WaypointError(f'invalid waypoint name {name!r}')
        try:
            x = float(x)
            y = float(y)
            z = float(z)
        except (TypeError, ValueError):
            raise WaypointError(
                f'{name!r}: position must be numbers, got '
                f'{x!r}, {y!r}, {z!r}')
        for v in (x, y, z):
            if not math.isfinite(v):
                raise WaypointError(f'{name!r}: non-finite position value')
        self.name = name.strip()
        self.x = x
        self.y = y
        self.z = z
        if max_linear_velocity is not None:
            max_linear_velocity = float(max_linear_velocity)
        if max_angular_velocity is not None:
            max_angular_velocity = float(max_angular_velocity)
        self.max_linear_velocity = max_linear_velocity
        self.max_angular_velocity = max_angular_velocity

    def to_dict(self):
        return {
            'name': self.name,
            'position': {'x': self.x, 'y': self.y, 'z': self.z},
        }

    @classmethod
    def from_dict(cls, entry, default_name):
        if not isinstance(entry, dict):
            raise WaypointError(f'waypoint entry not a mapping: {entry!r}')
        position = entry.get('position')
        if not isinstance(position, dict) or 'x' not in position \
                or 'y' not in position:
            raise WaypointError(
                f'{entry.get("name", default_name)!r}: position must be a '
                f'mapping with x and y')
        return cls(
            entry.get('name', default_name),
            position['x'], position['y'], position.get('z', 0.0),
            entry.get('max_linear_velocity'),
            entry.get('max_angular_velocity'))


def load_waypoints(path):
    """Load a waypoint yaml file. Returns a list of Waypoint.

    Tolerates the extra nav2 waypoint-follower fields (see module docstring).
    Raises WaypointError on a malformed file.
    """
    with open(path) as f:
        doc = yaml.safe_load(f)
    if not isinstance(doc, dict) or 'waypoints' not in doc:
        raise WaypointError(f'{path}: expected a mapping with a "waypoints" key')
    entries = doc['waypoints']
    if entries is None:
        entries = []
    if not isinstance(entries, list):
        raise WaypointError(f'{path}: "waypoints" must be a list')
    waypoints = []
    for i, entry in enumerate(entries):
        waypoints.append(Waypoint.from_dict(entry, default_name=f'wp{i + 1}'))
    return waypoints


def save_waypoints(path, waypoints):
    """Write waypoints to `path` (parent directories created on demand).

    Writes only name + position per waypoint, in the nav2 waypoint-follower
    format (see module docstring).
    """
    parent = os.path.dirname(os.path.abspath(path))
    os.makedirs(parent, exist_ok=True)
    doc = {'waypoints': [wp.to_dict() for wp in waypoints]}
    with open(path, 'w') as f:
        yaml.safe_dump(doc, f, sort_keys=False, default_flow_style=None)


def quaternion_from_yaw(yaw):
    """Map-frame yaw (rad) as a dict {x, y, z, w} (roll = pitch = 0)."""
    return {'x': 0.0, 'y': 0.0, 'z': math.sin(yaw / 2.0),
            'w': math.cos(yaw / 2.0)}


def yaw_to_face(current, target):
    """Yaw (rad) a robot at `current` (x, y indexable) needs to face
    `target` (x, y indexable)."""
    return math.atan2(target[1] - current[1], target[0] - current[0])


def goal_orientation(waypoints, index):
    """Orientation dict for waypoint `index`: face the NEXT waypoint when
    one exists, identity quaternion for the last one (see module docstring
    for why)."""
    if index + 1 < len(waypoints):
        current = waypoints[index]
        target = waypoints[index + 1]
        return quaternion_from_yaw(
            yaw_to_face((current.x, current.y), (target.x, target.y)))
    return dict(IDENTITY_QUATERNION)


class TeachSession:
    """Teach-mode state: recorded waypoints + save bookkeeping (pure)."""

    def __init__(self):
        self.waypoints = []
        self.saved_count = 0

    @property
    def dirty(self):
        return len(self.waypoints) != self.saved_count

    def record(self, x, y, z):
        """Append the current pose; names auto: wp1, wp2, ..."""
        wp = Waypoint(f'wp{len(self.waypoints) + 1}', x, y, z)
        self.waypoints.append(wp)
        return wp

    def undo(self):
        """Remove the last recorded waypoint; returns it, or None."""
        return self.waypoints.pop() if self.waypoints else None

    def save(self, path):
        save_waypoints(path, self.waypoints)
        self.saved_count = len(self.waypoints)
        return self.saved_count


class PlaybackState:
    """Retry state machine for one repeat run (pure; no ROS).

    Each waypoint gets one attempt plus up to `retries` retries (so
    retries + 1 attempts total). A failed final attempt marks the waypoint
    SKIPPED and the run moves to the next one. on_attempt_finished returns
    a (verdict, line) tuple where line is the exact live status text.
    """

    def __init__(self, waypoints, retries=2):
        self.waypoints = list(waypoints)
        self.total = len(self.waypoints)
        self.retries = int(retries)
        self.index = 0       # index of the waypoint in flight
        self.attempts = 0    # attempts spent on the current waypoint
        self.results = []    # (name, verdict, reason, attempts)
        self.done = self.total == 0

    def current(self):
        """Waypoint in flight, or None if the run is done."""
        return None if self.done else self.waypoints[self.index]

    def on_attempt_finished(self, succeeded, reason=''):
        """Feed one goal result for the current waypoint.

        Returns (verdict, line): verdict is REACHED / RETRY / SKIPPED and
        line is the live status text for this event.
        """
        wp = self.current()
        if wp is None:
            raise WaypointError('no waypoint in flight')
        self.attempts += 1
        attempts = self.attempts
        if succeeded:
            self.results.append((wp.name, REACHED, reason, attempts))
            self._advance()
            return REACHED, (
                f'{wp.name}/{self.total}: reached on attempt {attempts}')
        if self.attempts <= self.retries:
            return RETRY, (
                f'{wp.name}/{self.total}: retry '
                f'{self.attempts}/{self.retries} ({reason})')
        self.results.append((wp.name, SKIPPED, reason, attempts))
        self._advance()
        return SKIPPED, f'{wp.name}/{self.total}: SKIPPED ({reason})'

    def _advance(self):
        self.index += 1
        self.attempts = 0
        self.done = self.index >= self.total

    def summary(self):
        """Final run summary, e.g.
        'reached 8/10, skipped: wp3 (timeout), wp7 (aborted)'."""
        reached = [r for r in self.results if r[1] == REACHED]
        skipped = [r for r in self.results if r[1] == SKIPPED]
        line = f'reached {len(reached)}/{self.total}'
        if skipped:
            line += ', skipped: ' + ', '.join(
                f'{name} ({reason})' for name, _, reason, _ in skipped)
        return line


# ---------------------------------------------------------------------------
# ROS wiring (imports rclpy and message types; nothing above this line needs
# ROS, which is what lets the selftest extract and run the pure section on
# the Mac).
# ---------------------------------------------------------------------------

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from nav2_msgs.action import NavigateToPose

TEACH_HELP = """teach mode:
  SPACE   record current /amcl_pose as wpN
  U       undo last recorded waypoint
  S       save waypoints (yaml, nav2 waypoint-follower format)
  Q       quit
  H       this help"""

USAGE = """usage:
  ros2 run sentinel_bringup nav_waypoint_teach_repeat --teach
  ros2 run sentinel_bringup nav_waypoint_teach_repeat --repeat <file.yaml>

params (--ros-args -p <name>:=<value>):
  retries=2, goal_timeout=60.0, waypoint_dir=/home/sentinel1/dean/Sentinel/nav_waypoints"""

_STATUS_REASONS = {
    GoalStatus.STATUS_CANCELED: 'canceled',
    GoalStatus.STATUS_ABORTED: 'aborted',
}


def _status_reason(status):
    return _STATUS_REASONS.get(status, f'status {status}')


def _read_line(label):
    """Raw-mode line editor (cbreak kept on by the caller). Returns
    (line, True) or (None, False) if cancelled with ctrl+c. Same technique
    as arm_teach_console._read_line."""
    sys.stdout.write(f'\r{label}> ')
    sys.stdout.flush()
    buf = ''
    while True:
        ch = os.read(sys.stdin.fileno(), 1)
        if ch in (b'\r', b'\n'):
            sys.stdout.write('\n')
            return buf.strip(), True
        if ch in (b'\x7f', b'\x08'):  # backspace
            buf = buf[:-1]
        elif ch == b'\x15':           # ctrl+u: clear
            buf = ''
        elif ch == b'\x03':           # ctrl+c: cancel entry
            sys.stdout.write('\n')
            return None, False
        elif 32 <= ch[0] < 127:
            buf += ch.decode()
        sys.stdout.write(f'\r{label}> {buf}')
        sys.stdout.flush()


class NavWaypointNode(Node):

    def __init__(self, mode, repeat_file=None):
        super().__init__('nav_waypoint_teach_repeat')

        self.declare_parameter('retries', 2)
        self.declare_parameter('goal_timeout', 60.0)
        self.declare_parameter('waypoint_dir', DEFAULT_WAYPOINT_DIR)

        self.mode = mode
        self.retries = self.get_parameter('retries').value
        self.goal_timeout = float(self.get_parameter('goal_timeout').value)
        self.waypoint_dir = self.get_parameter('waypoint_dir').value
        self._in_flight = None
        # (send_fut, tracked) pairs for goal sends we abandoned on timeout:
        # a done-callback cancels the goal if the server accepts it late.
        self._pending_sends = []

        if mode == 'teach':
            self.session = TeachSession()
            self.latest_pose = None
            self.create_subscription(
                PoseWithCovarianceStamped, '/amcl_pose', self._amcl_cb, 10)
        else:
            self.state = PlaybackState(
                load_waypoints(self._resolve_path(repeat_file)),
                retries=self.retries)
            self._client = ActionClient(
                self, NavigateToPose, 'navigate_to_pose')

    def _amcl_cb(self, msg):
        self.latest_pose = msg

    def _resolve_path(self, filename):
        path = os.path.expanduser(filename)
        if not os.path.isabs(path):
            path = os.path.join(self.waypoint_dir, path)
        return path

    # ---- teach mode ----

    def run_teach(self):
        self.get_logger().info(
            'teach mode: SPACE record, U undo, S save, Q quit '
            f'(dir {self.waypoint_dir})')
        print(TEACH_HELP)
        old = termios.tcgetattr(sys.stdin.fileno())
        tty.setcbreak(sys.stdin.fileno())
        try:
            sys.stdout.write('\rteach> ')
            sys.stdout.flush()
            self._teach_loop()
        finally:
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old)
            print('\nbye')
            if self.session.dirty:
                unsaved = len(self.session.waypoints) - self.session.saved_count
                print(f'note: {unsaved} unsaved waypoint(s) discarded')

    def _teach_loop(self):
        while rclpy.ok():
            r, _, _ = select.select([sys.stdin], [], [], 0.05)
            rclpy.spin_once(self, timeout_sec=0.05)
            if not r:
                continue
            ch = os.read(sys.stdin.fileno(), 1)
            if ch in (b'\x03', b'\x04'):      # ctrl+c / ctrl+d
                break
            if ch == b'\x1b':                 # swallow arrow-key escapes
                self._swallow_escape()
                continue
            c = ch.decode()
            out = None
            if c == ' ':
                out = self._record()
            elif c in ('u', 'U'):
                out = self._undo()
            elif c in ('s', 'S'):
                out = self._save()
            elif c == 'q':
                break
            elif c == 'h':
                out = TEACH_HELP
            else:
                out = 'keys: SPACE record, U undo, S save, Q quit'
            if out:
                sys.stdout.write(f'\r{out}\n')
            sys.stdout.write('\rteach> ')
            sys.stdout.flush()

    def _swallow_escape(self):
        r2, _, _ = select.select([sys.stdin], [], [], 0.02)
        while r2:
            os.read(sys.stdin.fileno(), 1)
            r2, _, _ = select.select([sys.stdin], [], [], 0.02)

    def _record(self):
        if self.latest_pose is None:
            return 'no pose yet: /amcl_pose has not arrived (is AMCL up?)'
        p = self.latest_pose.pose.pose.position
        wp = self.session.record(p.x, p.y, p.z)
        return (f'{wp.name}: ({wp.x:.2f}, {wp.y:.2f}, {wp.z:.2f}) — '
                f'{len(self.session.waypoints)} waypoint(s)')

    def _undo(self):
        wp = self.session.undo()
        if wp is None:
            return 'nothing to undo'
        return (f'removed {wp.name} ({wp.x:.2f}, {wp.y:.2f}, {wp.z:.2f}) — '
                f'{len(self.session.waypoints)} waypoint(s)')

    def _save(self):
        filename, ok = _read_line(f'save as [{DEFAULT_WAYPOINT_FILE}]')
        if not ok:
            return 'save cancelled'
        filename = filename or DEFAULT_WAYPOINT_FILE
        path = self._resolve_path(filename)
        n = self.session.save(path)
        return f'saved {n} waypoint(s) to {path}'

    # ---- repeat mode ----

    def run_repeat(self):
        state = self.state
        if not state.waypoints:
            self.get_logger().error('no waypoints in file; nothing to repeat')
            return
        self.get_logger().info(
            f'repeat: {state.total} waypoint(s), retries={self.retries}, '
            f'goal_timeout={self.goal_timeout:.1f} s')
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                'navigate_to_pose action server (bt_navigator) not '
                'available — is the nav2 stack up?')
            return
        while rclpy.ok() and not state.done:
            wp = state.current()
            attempt = state.attempts + 1
            max_attempts = state.retries + 1
            print(f'{wp.name}/{state.total}: attempt {attempt}/{max_attempts} '
                  f'— goal ({wp.x:.2f}, {wp.y:.2f}, {wp.z:.2f})')
            succeeded, reason, detail = self._attempt(wp, state.index)
            _, line = state.on_attempt_finished(succeeded, reason)
            print(line)
            if detail:
                print(f'    bt_navigator: {detail}')
        print(state.summary())

    def _attempt(self, wp, index):
        """One NavigateToPose attempt on `wp`.

        Returns (succeeded, reason, detail): detail is extra bt_navigator
        error text ('' if none).

        Cancellation contract: no path may leave a goal able to keep
        executing on bt_navigator after we stop watching it. Goals timed
        out mid-flight are canceled and the cancellation is flushed before
        we return; a Ctrl+C mid-goal cancels and leaves self._in_flight set
        so cancel_in_flight() flushes the cancel before shutdown; goals
        accepted only after we gave up are canceled by a done-callback on
        the send future (fired by any later spin).
        """
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = wp.x
        goal.pose.pose.position.y = wp.y
        goal.pose.pose.position.z = wp.z
        goal.pose.pose.orientation = Quaternion(
            **goal_orientation(self.state.waypoints, index))

        tracked = {'abandoned': False}
        send_fut = self._client.send_goal_async(goal)
        self._pending_sends.append((send_fut, tracked))

        def _cancel_if_late(fut):
            # Goal accepted only after we abandoned the send on timeout:
            # cancel it before it can execute unwatched. Fires on whichever
            # spin happens after the late acceptance (a retry attempt's
            # loop, _cancel_and_flush, or cancel_in_flight).
            if not tracked['abandoned']:
                return
            try:
                handle = fut.result()
                if handle.accepted:
                    handle.cancel_goal_async()
            except Exception:
                self.get_logger().warn(
                    'late-accepted goal cancel failed; next goal preempts')

        send_fut.add_done_callback(_cancel_if_late)

        deadline = self.get_clock().now() + Duration(seconds=self.goal_timeout)
        while rclpy.ok() and not send_fut.done():
            if self.get_clock().now() > deadline:
                # Acceptance did not arrive in time. Give the server a short
                # grace to accept so we can cancel the goal ourselves; a
                # goal accepted even later is canceled by _cancel_if_late
                # (needs a spin, which every later attempt and
                # cancel_in_flight provide). Residual window: acceptance
                # later than the grace AND after the node has shut down —
                # bt_navigator taking >1 min to accept a goal is wedged,
                # and the goal would at worst drive to a taught waypoint.
                grace = deadline + Duration(seconds=2.0)
                while rclpy.ok() and not send_fut.done():
                    if self.get_clock().now() > grace:
                        tracked['abandoned'] = True
                        return False, 'timeout', ''
                    rclpy.spin_once(self, timeout_sec=0.05)
                break
            rclpy.spin_once(self, timeout_sec=0.05)
        handle = send_fut.result()
        if not handle.accepted:
            return False, 'rejected', ''
        if self.get_clock().now() > deadline:
            # Accepted only after the deadline: cancel it ourselves and
            # flush the cancellation before the retry.
            self._cancel_and_flush(handle)
            return False, 'timeout', ''
        self._in_flight = handle
        try:
            result_fut = handle.get_result_async()
            while rclpy.ok() and not result_fut.done():
                if self.get_clock().now() > deadline:
                    self._cancel_and_flush(handle)
                    self._in_flight = None
                    return False, 'timeout', ''
                rclpy.spin_once(self, timeout_sec=0.05)
            result = result_fut.result()
            self._in_flight = None
            # Outcome comes from the GOAL HANDLE status, not the result
            # message: on Humble NavigateToPose.result is std_msgs/Empty
            # (error_code/error_msg only exist on Iron+), while the handle
            # carries the terminal GoalStatus on every distro.
            status = handle.status
            if status == GoalStatus.STATUS_SUCCEEDED:
                return True, '', ''
            reason = _status_reason(status)
            detail = ''
            error_msg = getattr(result, 'error_msg', '')
            if error_msg:
                detail = (f'{reason}: {error_msg} '
                          f'(error_code {result.error_code})')
            return False, reason, detail
        except BaseException:
            # Ctrl+C (or any teardown exception) mid-goal: fire the cancel
            # now and KEEP self._in_flight set so main()'s finally can
            # flush the cancellation out to bt_navigator via
            # cancel_in_flight() before shutdown (re-canceling is
            # idempotent).
            handle.cancel_goal_async()
            raise

    def _cancel_and_flush(self, handle):
        """Cancel `handle` and spin until the cancellation is delivered
        (or a short deadline passes)."""
        handle.cancel_goal_async()
        end = self.get_clock().now() + Duration(seconds=2.0)
        while rclpy.ok() and self.get_clock().now() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def cancel_in_flight(self):
        """Cancel any goal still running (called on Ctrl+C / teardown) and
        spin the cancellation out to bt_navigator. The spin also fires
        late-acceptance done-callbacks so goals that accepted after we
        abandoned them get canceled."""
        handle = self._in_flight
        if handle is not None:
            handle.cancel_goal_async()
            self._in_flight = None
        # Mark every pending send abandoned BEFORE the flush spin: a goal
        # accepted during the flush must hit the _cancel_if_late path, not
        # the normal one — teardown leaves nobody to watch it.
        for _, tracked in self._pending_sends:
            tracked['abandoned'] = True
        if handle is None and not self._pending_sends:
            return
        end = self.get_clock().now() + Duration(seconds=2.0)
        while rclpy.ok() and self.get_clock().now() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        self._pending_sends[:] = [
            (fut, _) for fut, _ in self._pending_sends if not fut.done()]


def main(args=None):
    argv = list(sys.argv[1:] if args is None else args)
    mode = None
    repeat_file = None
    if '--teach' in argv:
        mode = 'teach'
    if '--repeat' in argv:
        mode = 'repeat'
        i = argv.index('--repeat')
        if i + 1 >= len(argv):
            print(USAGE)
            return 2
        repeat_file = argv[i + 1]
    if mode is None:
        print(USAGE)
        return 2

    rclpy.init(args=args)
    node = NavWaypointNode(mode, repeat_file)
    try:
        if mode == 'teach':
            node.run_teach()
        else:
            node.run_repeat()
    except KeyboardInterrupt:
        pass
    finally:
        node.cancel_in_flight()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
