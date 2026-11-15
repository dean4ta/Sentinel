#!/usr/bin/env python3
"""T011 selftest: goal_stress_test pure logic (no ROS).

Mirrors test_gamepad_twist_dedupe.py / test_nav_waypoint_teach_repeat.py:
the module under test imports rclpy at its 'ROS wiring' seam, so this test
never executes the whole module. It:

1. ast.parse()s the source with feature_version=(3, 10) — py310 grammar check.
2. Extracts everything ABOVE the first `import rclpy` statement (the pure
   section: stamp truncation, is_interesting, goal_from_pose orientation
   math, log/summary formatting, status names, bag command, defaults) and
   compiles it standalone.
3. Exercises the contracts with plain asserts.
4. Checks the launch file defaults (AST of the DeclareLaunchArgument
   calls) and py_compile-compiles the full module and the launch file
   (bytecode only; imports never run).

Run (Mac, no ROS needed):
  python3 test/test_goal_stress_test.py
"""

import ast
import math
import pathlib
import py_compile
import re
import types

HERE = pathlib.Path(__file__).resolve().parent
SRC = HERE.parent / 'sentinel_debug' / 'goal_stress_test.py'
LAUNCH = HERE.parent / 'launch' / 'goal_stress_test.launch.py'


def load_pure():
    """Compile and exec everything above the `import rclpy` seam."""
    source = SRC.read_text()
    tree = ast.parse(source, filename=str(SRC), feature_version=(3, 10))
    ros_idx = next(
        i for i, n in enumerate(tree.body)
        if isinstance(n, ast.Import) and any(a.name == 'rclpy' for a in n.names))
    assert ros_idx > 0, 'no pure section above the rclpy import seam'
    pure = ast.fix_missing_locations(
        ast.Module(body=tree.body[:ros_idx], type_ignores=[]))
    mod = types.ModuleType('_goal_stress_pure')
    code = compile(pure, str(SRC), 'exec')
    exec(code, mod.__dict__)
    return mod


def _status(sec, nanosec, code):
    """Stand-in for one action_msgs GoalStatus entry (no ROS imports)."""
    return types.SimpleNamespace(
        goal_info=types.SimpleNamespace(
            stamp=types.SimpleNamespace(sec=sec, nanosec=nanosec)),
        status=code)


def _quat(x, y, z, w):
    """Stand-in for a geometry_msgs Quaternion."""
    return types.SimpleNamespace(x=x, y=y, z=z, w=w)


def launch_defaults():
    """{arg_name: default} from the launch file's DeclareLaunchArgument
    calls (AST, no launch imports)."""
    tree = ast.parse(LAUNCH.read_text(), filename=str(LAUNCH),
                     feature_version=(3, 10))
    out = {}
    for node in ast.walk(tree):
        if (isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
                and node.func.id == 'DeclareLaunchArgument' and node.args):
            name = node.args[0].value
            val = next(
                (kw.value.value for kw in node.keywords
                 if kw.arg == 'default_value'), None)
            out[name] = val
    return out


def main():
    m = load_pure()
    truncated_stamp = m.truncated_stamp
    is_interesting = m.is_interesting
    format_log_line = m.format_log_line
    heartbeat_line = m.heartbeat_line
    status_name = m.status_name
    quat_from_yaw = m.quat_from_yaw
    yaw_from_quat = m.yaw_from_quat
    goal_from_pose = m.goal_from_pose
    default_bag_dir = m.default_bag_dir
    bag_command = m.bag_command
    latest_status = m.latest_status
    build_summary = m.build_summary
    BAG_TOPICS = m.BAG_TOPICS

    # --- defaults: healthy map-frame goal with a fresh stamp ---
    assert m.DEFAULT_FRAME == 'map'
    assert m.DEFAULT_TRUNCATE_STAMP is False
    assert m.DEFAULT_DISTANCE == 12.0
    assert m.DEFAULT_DURATION == 60.0
    assert m.FRESH_AMCL_SEC == 2.0
    assert m.AMCL_WAIT_SEC == 10.0
    defaults = launch_defaults()
    assert defaults.get('frame') == 'map'
    assert defaults.get('truncate_stamp') == 'false'
    assert defaults.get('duration') == '60.0'
    assert defaults.get('distance') == '12.0'

    # --- truncation math: nanosec -> 0, sec integer ---
    sec, nsec = truncated_stamp(1234, 987654321)
    assert (sec, nsec) == (1234, 0)
    assert isinstance(sec, int)
    assert isinstance(nsec, int)
    # float seconds truncate to the whole second too (Foxglove click stamp
    # can be up to ~1 s older than publish time).
    assert truncated_stamp(1234.999, 5) == (1234, 0)

    # --- is_interesting predicate (case-insensitive) ---
    assert is_interesting(
        'Lookup would require extrapolation into the future. '
        'Requested time 123.0...')
    assert is_interesting('EXTRAPOLATION Error in transformPoseInTargetFrame')
    assert is_interesting(
        '[planner_server]: Failed to transform pose in '
        'transformPoseInTargetFrame')
    assert is_interesting('[bt_navigator]: Aborting navigation')
    assert is_interesting('aborting because the goal is unreachable')
    # rejects noise
    assert not is_interesting('Received a goal, beginning planning')
    assert not is_interesting('trying to extrapolate the pose')  # no 'ion'
    assert not is_interesting('no abort happened')  # 'abort', not 'aborting'
    assert not is_interesting('')
    assert not is_interesting(None)
    assert not is_interesting(42)

    # --- quaternion/yaw helpers ---
    q = quat_from_yaw(0.0)
    assert q == {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
    q = quat_from_yaw(math.pi / 2)
    s = math.sqrt(2) / 2
    assert q['x'] == 0.0 and q['y'] == 0.0
    assert math.isclose(q['z'], s) and math.isclose(q['w'], s)
    assert yaw_from_quat(_quat(0.0, 0.0, 0.0, 1.0)) == 0.0
    for yaw in (0.0, 0.7, -1.2, math.pi, math.pi / 2):
        assert math.isclose(yaw_from_quat(_quat(**quat_from_yaw(yaw))), yaw)

    # --- goal_from_pose: 12 m ahead of the CURRENT pose ---
    g = goal_from_pose((1.0, 2.0, 0.0), 12.0, 'map')
    assert math.isclose(g['x'], 13.0) and math.isclose(g['y'], 2.0)
    assert g['frame'] == 'map'
    assert g['quat'] == {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
    g = goal_from_pose((0.0, 0.0, math.pi / 2), 12.0, 'map')
    assert math.isclose(g['x'], 0.0, abs_tol=1e-9)
    assert math.isclose(g['y'], 12.0)
    s = math.sqrt(2) / 2
    assert g['quat']['x'] == 0.0 and g['quat']['y'] == 0.0
    assert math.isclose(g['quat']['z'], s)
    assert math.isclose(g['quat']['w'], s)
    g = goal_from_pose((3.0, 4.0, math.pi), 12.0, 'map')
    assert math.isclose(g['x'], 3.0 - 12.0)
    assert math.isclose(g['y'], 4.0, abs_tol=1e-9)
    yaw = 0.7
    g = goal_from_pose((1.5, -2.5, yaw), 12.0, 'map')
    assert math.isclose(g['x'], 1.5 + 12.0 * math.cos(yaw))
    assert math.isclose(g['y'], -2.5 + 12.0 * math.sin(yaw))
    # base_link: ahead of the robot regardless of the map pose
    g = goal_from_pose((99.0, -99.0, 1.3), 12.0, 'base_link')
    assert g['x'] == 12.0 and g['y'] == 0.0 and g['frame'] == 'base_link'
    assert g['quat'] == {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
    # unknown frame rejected
    try:
        goal_from_pose((0.0, 0.0, 0.0), 12.0, 'odom')
        raise AssertionError('unknown frame did not raise')
    except ValueError:
        pass

    # --- log line format: stamp + node name prefix ---
    line = format_log_line((123, 4), 'bt_navigator', 'hello')
    assert line == '[123.000000004] bt_navigator: hello'
    line = format_log_line((7, 0), 'n', 'x')
    assert line == '[7.000000000] n: x'

    # --- heartbeat line ---
    assert heartbeat_line(25.03, 'EXECUTING', 3.4) == \
        '[t=+25.0s] state=EXECUTING dist=3.40m'
    assert heartbeat_line(25.03, '-', None) == '[t=+25.0s] state=-'

    # --- status names ---
    assert status_name(0) == 'UNKNOWN'
    assert status_name(4) == 'SUCCEEDED'
    assert status_name(6) == 'ABORTED'
    assert status_name(99) == 'status 99'

    # --- bag command + topics ---
    cmd = bag_command('/tmp/bagdir')
    assert cmd == ['ros2', 'bag', 'record', '-o', '/tmp/bagdir'] + BAG_TOPICS
    assert len(BAG_TOPICS) == 8
    for t in ('/goal_pose', '/tf', '/tf_static', '/plan', '/cmd_vel',
              '/amcl_pose', '/navigate_to_pose/_action/status', '/rosout'):
        assert t in BAG_TOPICS

    # --- newest action status wins ---
    assert latest_status([]) is None
    assert latest_status([_status(1, 0, 2)]) .status == 2
    assert latest_status(
        [_status(1, 0, 2), _status(2, 0, 6)]).status == 6
    # same stamp: first entry wins (stable)
    assert latest_status(
        [_status(1, 0, 2), _status(1, 0, 6)]).status == 2

    # --- default bag dir shape: ~/bags/goal_stress_<timestamp> ---
    d = default_bag_dir()
    assert d.startswith(str(pathlib.Path.home()) + '/bags/goal_stress_')
    assert re.fullmatch(r'.*_20\d{6}_\d{6}', d), d

    # --- summary text ---
    s = build_summary(
        goal_frame='base_link', truncated=True, distance=12.0,
        duration=60.0, goal_published=True, extrapolation_seen=True,
        first_interesting_wall='14:03:22', final_status='ABORTED',
        rosout_path='/tmp/x/rosout_filtered.txt', bag_dir='/tmp/x')
    assert 'goal: published x=+12.0 m in base_link' in s
    assert 'truncated whole-second (Foxglove mimic)' in s
    assert 'duration: 60.0 s' in s
    assert 'extrapolation seen: YES' in s
    assert 'first signature line at 14:03:22' in s
    assert 'final navigate_to_pose status: ABORTED' in s
    assert '/tmp/x/rosout_filtered.txt' in s
    assert 'bag dir: /tmp/x' in s

    s2 = build_summary(
        goal_frame='map', truncated=False, distance=12.0, duration=60.0,
        goal_published=True, extrapolation_seen=False,
        first_interesting_wall=None, final_status='SUCCEEDED',
        rosout_path='/tmp/y/r.txt', bag_dir='/tmp/y')
    assert 'goal: published x=+12.0 m in map' in s2
    assert 'fresh current time (healthy default)' in s2
    assert 'extrapolation seen: no' in s2
    assert '14:03:22' not in s2
    assert 'final navigate_to_pose status: SUCCEEDED' in s2

    # refusal path: no fresh amcl -> goal not published, stated plainly
    s3 = build_summary(
        goal_frame='map', truncated=False, distance=12.0, duration=60.0,
        goal_published=False, extrapolation_seen=False,
        first_interesting_wall=None, final_status='(no status seen)',
        rosout_path='/tmp/z/r.txt', bag_dir='/tmp/z')
    assert 'goal: NOT published (no fresh /amcl_pose)' in s3

    # --- py310 grammar + bytecode: module and launch file ---
    py_compile.compile(str(SRC), doraise=True)
    py_compile.compile(str(LAUNCH), doraise=True)

    print('test_goal_stress_test.py: ALL PASS')


if __name__ == '__main__':
    main()
