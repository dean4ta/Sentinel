#!/usr/bin/env python3
"""T011 goal stress test: Foxglove /goal_pose failure modes vs healthy control.

What this runs
--------------
Publishes ONE /goal_pose while bagging + monitoring the evidence topics.
The default run is HEALTHY: the goal is `distance` m ahead of the robot's
CURRENT pose — taken from TF (map <- base_link, latest time), with the
latest /amcl_pose as a regardless-of-age fallback — in the map frame,
along the current heading, with a fresh stamp; the robot really drives
`distance` m. The Foxglove mimic is explicit: frame:=base_link
truncate_stamp:=true.

Root-cause findings (first on-robot run + supervisor diagnosis)
---------------------------------------------------------------
1. IMMEDIATE recovery — the bug as observed on the first run: Foxglove's
   click stamp is a whole-second truncation (nanosec = 0), so it is up to
   ~1 s in the past. planner_server's transform_tolerance is 0.2 s, so
   the FIRST plan attempt re-transforms that stamp and fails with an
   "Extrapolation Error" instantly -> immediate recovery. This masks the
   10 s story below.
2. TF-buffer death: a base_link goal with a FRESH stamp re-transforms the
   original stamped goal on every ~1 s replan, and after ~10 s the stamp
   falls out of the TF buffer (nav2 default) -> "Extrapolation Error" ->
   planner abort -> recoveries -> "Goal failed".

Pose source
-----------
TF, not /amcl_pose freshness: AMCL stops publishing /amcl_pose when the
robot is stationary (update gates), so a freshness gate wrongly refuses
to publish. The goal pose comes from lookupTransform('map', 'base_link',
Time(0)); if TF is unavailable the latest /amcl_pose is used regardless
of age (warned when older than STALE_AMCL_SEC); publishing is refused
only when NEITHER source exists after AMCL_WAIT_SEC.

Evidence
--------
* `ros2 bag record` (CLI subprocess only, no rosbag2 python dependency)
  captures /goal_pose /tf /tf_static /plan /cmd_vel /amcl_pose
  /navigate_to_pose/_action/status /rosout, into
  ~/bags/goal_stress_<timestamp>_<pid> (deduplicated with -2/-3 suffixes),
* /rosout is filtered to <bag_dir>/rosout_filtered.txt (every line,
  prefixed with its stamp + node name); any line matching 'Extrapolation'
  / 'transformPoseInTargetFrame' / 'Aborting' (case-insensitive) prints
  live,
* /navigate_to_pose/_action/status transitions are logged with timestamps
  and a heartbeat prints every 5 s (state + distance driven, from TF).

Run (bringup already up):
  ros2 launch sentinel_debug goal_stress_test.launch.py   # healthy default
  ros2 launch sentinel_debug goal_stress_test.launch.py \
      frame:=base_link truncate_stamp:=true               # Foxglove mimic
  ros2 launch sentinel_debug goal_stress_test.launch.py \
      frame:=base_link truncate_stamp:=false   # isolates the ~10 s death

Everything above the 'ROS wiring' seam below is pure Python (no rclpy), so
the logic tests on the Mac without ROS.
"""

import math
import os
import time

DEFAULT_FRAME = 'map'
DEFAULT_TRUNCATE_STAMP = False
DEFAULT_DISTANCE = 12.0
DEFAULT_DURATION = 60.0

STALE_AMCL_SEC = 5.0   # fallback /amcl_pose older than this gets a warning
AMCL_WAIT_SEC = 10.0   # how long to wait for a pose source before refusing

BAG_TOPICS = [
    '/goal_pose',
    '/tf',
    '/tf_static',
    '/plan',
    '/cmd_vel',
    '/amcl_pose',
    '/navigate_to_pose/_action/status',
    '/rosout',
]

# action_msgs/GoalStatus codes (plain ints; the ROS wiring imports the
# real constants, these names are for display).
STATUS_NAMES = {
    0: 'UNKNOWN',
    1: 'ACCEPTED',
    2: 'EXECUTING',
    3: 'CANCELING',
    4: 'SUCCEEDED',
    5: 'CANCELED',
    6: 'ABORTED',
}

# Substrings that identify the TF-buffer-death signature in a /rosout line
# (matched case-insensitively).
INTERESTING_SUBSTRINGS = (
    'extrapolation',
    'transformposeintargetframe',
    'aborting',
)

IDENTITY_QUATERNION = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}


def truncated_stamp(sec, nanosec):
    """Whole-second truncation of a stamp: returns (int(sec), 0).

    Exactly what Foxglove's click timestamp looks like: sec only, nanosec
    zero. The stamp is then up to ~1 s older than publish time — already
    past planner_server's 0.2 s transform_tolerance, which is what fails
    the first plan attempt instantly (see module docstring).
    """
    return (int(sec), 0)


def is_interesting(line):
    """True if `line` carries the TF-buffer-death diagnosis signature
    ('Extrapolation', 'transformPoseInTargetFrame' or 'Aborting',
    case-insensitive). Non-strings are never interesting."""
    if not isinstance(line, str):
        return False
    lowered = line.lower()
    return any(s in lowered for s in INTERESTING_SUBSTRINGS)


def format_log_line(stamp, node_name, text):
    """'[sec.nanosec] node_name: text' — the rosout_filtered.txt prefix."""
    return f'[{stamp[0]}.{stamp[1]:09d}] {node_name}: {text}'


def heartbeat_line(elapsed, state_name, distance_m):
    """'[t=+12.5s] state=EXECUTING dist=3.42m' (dist omitted when unknown)."""
    parts = [f'[t=+{elapsed:.1f}s] state={state_name}']
    if distance_m is not None:
        parts.append(f'dist={distance_m:.2f}m')
    return ' '.join(parts)


def status_name(code):
    """action_msgs GoalStatus code -> display name."""
    return STATUS_NAMES.get(code, f'status {code}')


def quat_from_yaw(yaw):
    """Yaw (rad) as a dict {x, y, z, w} (roll = pitch = 0)."""
    return {'x': 0.0, 'y': 0.0,
            'z': math.sin(yaw / 2.0), 'w': math.cos(yaw / 2.0)}


def yaw_from_quat(q):
    """Yaw (rad) from a duck-typed quaternion (x, y, z, w attributes)."""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def goal_from_pose(pose, distance, frame):
    """Goal spec for `distance` m ahead of the robot's CURRENT pose.

    pose: (x, y, yaw) — the current robot pose (normally TF-derived).
    frame 'map': x + distance*cos(yaw), y + distance*sin(yaw), quaternion
      from yaw, frame 'map' — the healthy default the planner can consume
      directly (no re-transform of a stale stamp).
    frame 'base_link': x = distance, y = 0, identity quaternion, frame
      'base_link' — ahead of the robot regardless of the map pose (what
      Foxglove publishes).

    Returns {'x', 'y', 'quat', 'frame'} (quat is a dict). Any other frame
    raises ValueError.
    """
    if frame == 'base_link':
        return {'x': float(distance), 'y': 0.0,
                'quat': dict(IDENTITY_QUATERNION), 'frame': 'base_link'}
    if frame != 'map':
        raise ValueError(f'unknown goal frame {frame!r}')
    x, y, yaw = pose
    return {'x': x + distance * math.cos(yaw),
            'y': y + distance * math.sin(yaw),
            'quat': quat_from_yaw(yaw), 'frame': 'map'}


def choose_pose(tf_ok, amcl_age):
    """Pick the goal-pose source (pure decision).

    tf_ok: a TF lookup succeeded (map <- base_link, latest time).
    amcl_age: age in seconds of the latest /amcl_pose message, or None if
    no message ever arrived.
    Returns (source, warning): source is 'tf' / 'amcl' / None (refuse),
    warning is text for the caller to log, or None.
    """
    if tf_ok:
        return ('tf', None)
    if amcl_age is not None:
        if amcl_age > STALE_AMCL_SEC:
            return ('amcl', f'/amcl_pose fallback is stale '
                            f'({amcl_age:.1f} s old); using it anyway')
        return ('amcl', None)
    return (None, 'no TF (map <- base_link) and no /amcl_pose — '
                  'refusing to publish the goal')


def default_bag_dir():
    """~/bags/goal_stress_<YYYYmmdd_HHMMSS>_<pid> — the PID keeps two runs
    started in the same second distinct."""
    return os.path.expanduser(
        '~/bags/goal_stress_' + time.strftime('%Y%m%d_%H%M%S')
        + f'_{os.getpid()}')


def unique_bag_dir(base):
    """`base` if nothing exists there yet, else base-2, base-3, ... —
    bag dirs must never collide (rosbag2 would append into an existing
    dir instead of failing)."""
    candidate = base
    n = 1
    while os.path.exists(candidate):
        n += 1
        candidate = f'{base}-{n}'
    return candidate


def bag_command(bag_dir):
    """Argv for the `ros2 bag record` subprocess."""
    return ['ros2', 'bag', 'record', '-o', bag_dir] + BAG_TOPICS


def latest_status(status_list):
    """Newest entry of a GoalStatusArray.status_list (duck-typed: entries
    with .goal_info.stamp.sec/.nanosec and .status), or None if empty."""
    if not status_list:
        return None
    latest = None
    latest_stamp = None
    for st in status_list:
        s = st.goal_info.stamp
        if latest_stamp is None or (s.sec, s.nanosec) > latest_stamp:
            latest_stamp = (s.sec, s.nanosec)
            latest = st
    return latest


def build_summary(goal_frame, truncated, distance, duration, goal_published,
                  extrapolation_seen, first_interesting_wall, final_status,
                  rosout_path, bag_dir):
    """End-of-run summary text (pure; all inputs are plain values)."""
    stamp_mode = ('truncated whole-second (Foxglove mimic)'
                  if truncated else 'fresh current time (healthy default)')
    if goal_published:
        goal_line = f'  goal: published x=+{distance} m in {goal_frame}'
    else:
        goal_line = ('  goal: NOT published '
                     '(no pose source: TF and /amcl_pose unavailable)')
    lines = [
        'goal stress test summary',
        goal_line,
        f'  stamp: {stamp_mode}',
        f'  duration: {duration} s',
    ]
    first = (f' (first signature line at {first_interesting_wall})'
             if first_interesting_wall else '')
    if extrapolation_seen:
        lines.append(f'  extrapolation seen: YES{first}')
    else:
        lines.append(f'  extrapolation seen: no{first}')
    lines.append(f'  final navigate_to_pose status: {final_status}')
    lines.append(f'  rosout log: {rosout_path}')
    lines.append(f'  bag dir: {bag_dir}')
    return '\n'.join(lines)


# ---------------------------------------------------------------------------
# ROS wiring (imports rclpy and message types; nothing above this line needs
# ROS, which is what lets the selftest extract and run the pure section on
# the Mac).
# ---------------------------------------------------------------------------

import signal
import subprocess

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion
from rcl_interfaces.msg import Log
from tf2_ros import Buffer, TransformListener

BAG_WARMUP_SEC = 2.0
HEARTBEAT_SEC = 5.0


class GoalStressNode(Node):

    def __init__(self):
        super().__init__('goal_stress_test')

        self.declare_parameter('distance', DEFAULT_DISTANCE)
        self.declare_parameter('frame', DEFAULT_FRAME)
        self.declare_parameter('truncate_stamp', DEFAULT_TRUNCATE_STAMP)
        self.declare_parameter('duration', DEFAULT_DURATION)
        self.declare_parameter('bag_dir', '')

        self.distance = float(self.get_parameter('distance').value)
        self.frame = str(self.get_parameter('frame').value)
        self.truncate_stamp = bool(self.get_parameter('truncate_stamp').value)
        self.duration = float(self.get_parameter('duration').value)
        bag_dir = str(self.get_parameter('bag_dir').value)
        self.bag_dir = bag_dir or unique_bag_dir(default_bag_dir())
        self.rosout_path = os.path.join(self.bag_dir, 'rosout_filtered.txt')

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        # /amcl_pose is kept ONLY as the fallback goal-pose source (TF is
        # primary); heartbeat distance uses TF as well.
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._amcl_cb, 10)
        # /rosout depth 100: the bringup is chatty; drop nothing relevant.
        self.create_subscription(Log, '/rosout', self._rosout_cb, 100)
        self.create_subscription(
            GoalStatusArray, '/navigate_to_pose/_action/status',
            self._action_cb, 10)

        self._log_file = None
        self._log_write_failed = False
        self._bag_proc = None
        self._goal_published = False
        self._amcl_msg = None
        self._extrapolation_seen = False
        self._first_interesting_wall = None
        self._last_status = None
        self._start_pos = None
        self._t0 = time.monotonic()
        self._last_heartbeat = self._t0

    # ---- subscriptions ----

    def _amcl_cb(self, msg):
        self._amcl_msg = msg

    def _rosout_cb(self, msg):
        # Skip our own log lines: echoing an interesting line through the
        # logger would loop forever (logger -> /rosout -> this callback).
        if msg.name == self.get_name():
            return
        line = format_log_line(
            (msg.stamp.sec, msg.stamp.nanosec), msg.name, msg.msg)
        if self._log_file is not None:
            try:
                self._log_file.write(line + '\n')
                self._log_file.flush()
            except OSError:
                if not self._log_write_failed:
                    self._log_write_failed = True
                    print(f'*** WARNING: cannot write {self.rosout_path}')
        if is_interesting(msg.msg):
            wall = time.strftime('%H:%M:%S')
            if self._first_interesting_wall is None:
                self._first_interesting_wall = wall
            if 'extrapolation' in msg.msg.lower():
                self._extrapolation_seen = True
            print(f'*** {line}')

    def _action_cb(self, msg):
        st = latest_status(msg.status_list)
        if st is None:
            return
        name = status_name(st.status)
        if name == self._last_status:
            return
        prev = self._last_status or '-'
        self._last_status = name
        print(f'[t=+{time.monotonic() - self._t0:.1f}s] '
              f'navigate_to_pose: {prev} -> {name}')

    # ---- run ----

    def _tf_pose_xy(self):
        """Current (x, y) from TF (map <- base_link, latest), or None."""
        try:
            tf = self._tf_buffer.lookup_transform('map', 'base_link', Time())
        except Exception:
            return None
        return (tf.transform.translation.x, tf.transform.translation.y)

    def _heartbeat(self):
        dist = None
        pos = self._tf_pose_xy()
        if self._start_pos is not None and pos is not None:
            dist = math.hypot(
                pos[0] - self._start_pos[0], pos[1] - self._start_pos[1])
        state = self._last_status or '(no status yet)'
        print(heartbeat_line(time.monotonic() - self._t0, state, dist))

    def _amcl_age_sec(self):
        """Age of the latest /amcl_pose stamp in seconds (>= 0)."""
        now = self.get_clock().now()
        stamp = Time.from_msg(self._amcl_msg.header.stamp)
        return max(0.0, (now - stamp).nanoseconds / 1e9)

    def _goal_pose_sources(self):
        """Pose source: TF first (map <- base_link, latest), /amcl_pose
        fallback. Returns (pose, source, warning): pose is (x, y, yaw) or
        None (refuse)."""
        try:
            tf = self._tf_buffer.lookup_transform('map', 'base_link', Time())
        except Exception:
            tf = None
        if tf is not None:
            tr = tf.transform
            return ((tr.translation.x, tr.translation.y,
                     yaw_from_quat(tr.rotation)), 'tf', None)
        amcl_age = (self._amcl_age_sec()
                    if self._amcl_msg is not None else None)
        source, warning = choose_pose(False, amcl_age)
        if source == 'amcl':
            p = self._amcl_msg.pose.pose
            return ((p.position.x, p.position.y,
                     yaw_from_quat(p.orientation)), 'amcl', warning)
        return (None, None, warning)

    def _wait_goal_pose(self):
        """Spin up to AMCL_WAIT_SEC for TF (map <- base_link); after the
        window fall back to /amcl_pose regardless of age. Refuse only if
        NEITHER source is available. Returns (pose, source, warning)."""
        deadline = time.monotonic() + AMCL_WAIT_SEC
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            pose, source, warning = self._goal_pose_sources()
            if source == 'tf':
                return pose, source, warning
        return self._goal_pose_sources()

    def _publish_goal(self, pose, source):
        """Publish the ONE goal `distance` m ahead of the robot's CURRENT
        pose (map frame) or ahead of the robot itself (base_link), with
        an optional whole-second stamp truncation. Published exactly
        once: a second publish would reset the evidence clock and mask
        the bug."""
        spec = goal_from_pose(pose, self.distance, self.frame)
        now = self.get_clock().now().to_msg()
        goal = PoseStamped()
        goal.header.frame_id = spec['frame']
        if self.truncate_stamp:
            goal.header.stamp.sec, goal.header.stamp.nanosec = \
                truncated_stamp(now.sec, now.nanosec)
        else:
            goal.header.stamp = now
        goal.pose.position.x = spec['x']
        goal.pose.position.y = spec['y']
        goal.pose.position.z = 0.0
        goal.pose.orientation = Quaternion(**spec['quat'])
        self._goal_pub.publish(goal)
        self._goal_published = True
        self._start_pos = (pose[0], pose[1])  # heartbeat distance resets here
        self.get_logger().info(
            f'published /goal_pose: ({spec["x"]:.2f}, {spec["y"]:.2f}) in '
            f'{spec["frame"]} from {source} pose '
            f'({pose[0]:.2f}, {pose[1]:.2f}), yaw {pose[2]:.2f}), stamp '
            f'{goal.header.stamp.sec}.{goal.header.stamp.nanosec:09d} '
            f'({"truncated" if self.truncate_stamp else "fresh"})')

    def run(self):
        os.makedirs(self.bag_dir, exist_ok=True)
        self._log_file = open(self.rosout_path, 'w')
        self._bag_proc = subprocess.Popen(bag_command(self.bag_dir))

        self.get_logger().info(
            f'goal stress test: distance={self.distance} m, '
            f'frame={self.frame}, truncate_stamp={self.truncate_stamp}, '
            f'duration={self.duration} s')
        self.get_logger().info('bag: ' + ' '.join(bag_command(self.bag_dir)))
        self.get_logger().info(f'rosout filter: {self.rosout_path}')

        # Warm-up: let the bag start writing and subscriptions connect.
        start = self._t0
        while rclpy.ok() and time.monotonic() - start < BAG_WARMUP_SEC:
            rclpy.spin_once(self, timeout_sec=0.05)
        if self._bag_proc.poll() is not None:
            self.get_logger().warn(
                f'ros2 bag exited early (rc={self._bag_proc.returncode}); '
                'continuing without a bag')
        if self._goal_pub.get_subscription_count() == 0:
            self.get_logger().warn(
                'no subscribers on /goal_pose; the goal may be missed '
                '(is bt_navigator up?)')

        # The goal is `distance` m ahead of the robot's CURRENT pose:
        # TF (map <- base_link) is the live source — AMCL stops
        # publishing /amcl_pose when the robot is stationary (update
        # gates) — with the latest /amcl_pose as a regardless-of-age
        # fallback. Refuse only if neither exists.
        pose, source, warning = self._wait_goal_pose()
        if pose is None:
            self.get_logger().warn(
                (warning or 'no pose source') + ' — REFUSING to publish '
                'the goal; monitoring continues without a goal')
        else:
            if warning:
                self.get_logger().warn(warning)
            self._publish_goal(pose, source)

        while rclpy.ok() and time.monotonic() - start < self.duration:
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.monotonic() - self._last_heartbeat >= HEARTBEAT_SEC:
                self._last_heartbeat = time.monotonic()
                self._heartbeat()

    def stop_bag(self):
        """SIGINT the bag subprocess and wait for a clean exit (no hanging
        subprocess), then close the filtered rosout file."""
        proc = self._bag_proc
        if proc is not None and proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                self.get_logger().warn('ros2 bag ignored SIGINT; SIGTERM')
                proc.terminate()
                try:
                    proc.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait()
        if self._log_file is not None:
            self._log_file.close()
            self._log_file = None

    def print_summary(self):
        print(build_summary(
            goal_frame=self.frame,
            truncated=self.truncate_stamp,
            distance=self.distance,
            duration=self.duration,
            goal_published=self._goal_published,
            extrapolation_seen=self._extrapolation_seen,
            first_interesting_wall=self._first_interesting_wall,
            final_status=self._last_status or '(no status seen)',
            rosout_path=self.rosout_path,
            bag_dir=self.bag_dir))


def main(args=None):
    rclpy.init(args=args)
    node = GoalStressNode()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('interrupted; stopping bag and summarizing')
    finally:
        node.stop_bag()
        node.print_summary()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
