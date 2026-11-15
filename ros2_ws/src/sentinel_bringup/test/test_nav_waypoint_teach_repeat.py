#!/usr/bin/env python3
"""T010 selftest: nav_waypoint_teach_repeat pure logic (no ROS).

Mirrors test_gamepad_twist_dedupe.py: the module under test imports rclpy at
its 'ROS wiring' seam, so this test never executes the whole module. It:

1. ast.parse()s the source with feature_version=(3, 10) — py310 grammar check.
2. Extracts everything ABOVE the first `import rclpy` statement (the pure
   section: Waypoint, load/save, quaternion helpers, TeachSession, the retry
   state machine) and compiles it standalone.
3. Exercises the contracts with plain asserts.
4. py_compile-compiles the full module (bytecode only; imports never run).

Run (Mac, no ROS needed):
  uv run --with pyyaml python3 test/test_nav_waypoint_teach_repeat.py
"""

import ast
import math
import pathlib
import py_compile
import tempfile
import types

HERE = pathlib.Path(__file__).resolve().parent
SRC = HERE.parent / 'sentinel_bringup' / 'nav_waypoint_teach_repeat.py'

try:
    import yaml  # noqa: F401  (pure section needs it at exec time)
except ImportError:
    raise SystemExit(
        'PyYAML not available: run with\n'
        '  uv run --with pyyaml python3 test/test_nav_waypoint_teach_repeat.py')


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
    mod = types.ModuleType('_nav_wpt_pure')
    code = compile(pure, str(SRC), 'exec')
    exec(code, mod.__dict__)
    return mod


def _wps(cls, names, positions):
    return [cls(n, *p) for n, p in zip(names, positions)]


def main():
    m = load_pure()
    Waypoint = m.Waypoint
    WaypointError = m.WaypointError
    load_waypoints = m.load_waypoints
    save_waypoints = m.save_waypoints
    TeachSession = m.TeachSession
    PlaybackState = m.PlaybackState
    REACHED, RETRY, SKIPPED = m.REACHED, m.RETRY, m.SKIPPED
    goal_orientation = m.goal_orientation
    quaternion_from_yaw = m.quaternion_from_yaw
    yaw_to_face = m.yaw_to_face

    # --- yaml save/load round-trip ---
    with tempfile.TemporaryDirectory() as tmp:
        path = pathlib.Path(tmp) / 'nested' / 'wps.yaml'  # save mkdirs parents
        wps = [Waypoint('wp1', 1.0, 2.0, 0.0), Waypoint('wp2', -3.5, 4.25)]
        save_waypoints(path, wps)
        text = path.read_text()
        assert 'waypoints:' in text
        assert 'position:' in text
        assert 'x: 1.0' in text
        # we never write the nav2 velocity fields
        assert 'max_linear_velocity' not in text
        assert 'max_angular_velocity' not in text
        back = load_waypoints(path)
        assert len(back) == 2
        for a, b in zip(wps, back):
            assert b.name == a.name
            assert (b.x, b.y, b.z) == (a.x, a.y, a.z)

    # --- loader tolerance for extra nav2 waypoint-follower fields ---
    with tempfile.TemporaryDirectory() as tmp:
        path = pathlib.Path(tmp) / 'nav2_file.yaml'
        path.write_text(
            'waypoints:\n'
            '- name: a\n'
            '  position:\n'
            '    x: 1.5\n'
            '    y: 2.5\n'
            '    z: 0.1\n'
            '  max_linear_velocity: 0.26\n'
            '  max_angular_velocity: 1.0\n'
            '- name: b\n'                        # z omitted -> 0.0
            '  position: {x: 3.0, y: 4.0}\n'
            '- position: {x: 5.0, y: 6.0}\n'     # name omitted -> wp3
            '  some_unknown_field: ignored\n')
        loaded = load_waypoints(path)
        assert [w.name for w in loaded] == ['a', 'b', 'wp3']
        assert (loaded[0].x, loaded[0].y, loaded[0].z) == (1.5, 2.5, 0.1)
        assert loaded[0].max_linear_velocity == 0.26
        assert loaded[0].max_angular_velocity == 1.0
        assert loaded[1].z == 0.0
        assert loaded[1].max_linear_velocity is None
        assert (loaded[2].x, loaded[2].y) == (5.0, 6.0)
        # round-trip of a nav2 file drops the velocity fields
        save_waypoints(path, loaded)
        assert 'max_linear_velocity' not in path.read_text()

    # --- malformed files raise WaypointError ---
    with tempfile.TemporaryDirectory() as tmp:
        p1 = pathlib.Path(tmp) / 'bad1.yaml'
        p1.write_text('not_waypoints: []\n')
        try:
            load_waypoints(p1)
            assert False, 'missing waypoints key should raise'
        except WaypointError:
            pass
        p2 = pathlib.Path(tmp) / 'bad2.yaml'
        p2.write_text('waypoints:\n- position: [1.0, 2.0]\n')  # not a mapping
        try:
            load_waypoints(p2)
            assert False, 'list position should raise'
        except WaypointError:
            pass

    # --- retry state machine: reached first try ---
    st = PlaybackState(_wps(Waypoint, ['wp1', 'wp2', 'wp3'], [(0, 0), (1, 0), (2, 0)]),
                       retries=2)
    v, line = st.on_attempt_finished(True)
    assert v == REACHED and line == 'wp1/3: reached on attempt 1'
    assert st.index == 1 and st.attempts == 0 and not st.done

    # --- fail -> retry -> fail -> retry -> fail -> skip (3 attempts total) ---
    st = PlaybackState(_wps(Waypoint, ['wp1', 'wp2'], [(0, 0), (1, 0)]), retries=2)
    v, line = st.on_attempt_finished(False, 'timeout')
    assert v == RETRY and line == 'wp1/2: retry 1/2 (timeout)'
    assert st.attempts == 1 and st.index == 0
    v, line = st.on_attempt_finished(False, 'timeout')
    assert v == RETRY and line == 'wp1/2: retry 2/2 (timeout)'
    v, line = st.on_attempt_finished(False, 'timeout')
    assert v == SKIPPED and line == 'wp1/2: SKIPPED (timeout)'
    assert st.index == 1 and st.attempts == 0 and not st.done

    # --- fail -> retry -> reached ---
    st = PlaybackState(_wps(Waypoint, ['wp1'], [(0, 0)]), retries=2)
    v, _ = st.on_attempt_finished(False, 'aborted')
    assert v == RETRY
    v, line = st.on_attempt_finished(True)
    assert v == REACHED and st.done
    assert line == 'wp1/1: reached on attempt 2'

    # --- retries=0 is the old no-retry behavior: first failure skips ---
    st = PlaybackState(_wps(Waypoint, ['wp1'], [(0, 0)]), retries=0)
    v, line = st.on_attempt_finished(False, 'timeout')
    assert v == SKIPPED and line == 'wp1/1: SKIPPED (timeout)'
    assert st.done

    # --- retry counter resets per waypoint ---
    st = PlaybackState(_wps(Waypoint, ['wp1', 'wp2', 'wp3'], [(0, 0), (1, 0), (2, 0)]),
                       retries=2)
    st.on_attempt_finished(False, 'x')              # wp1 retry 1/2
    st.on_attempt_finished(False, 'x')              # wp1 retry 2/2
    v, _ = st.on_attempt_finished(False, 'x')       # wp1 skipped
    assert v == SKIPPED and st.attempts == 0
    v, line = st.on_attempt_finished(False, 'timeout')  # wp2 starts fresh
    assert v == RETRY and line == 'wp2/3: retry 1/2 (timeout)'

    # --- summary text ---
    st = PlaybackState(_wps(Waypoint, ['wp1', 'wp2', 'wp3'], [(0, 0), (1, 0), (2, 0)]),
                       retries=1)
    st.on_attempt_finished(True)                    # wp1 reached
    st.on_attempt_finished(False, 'timeout')        # wp2 retry 1/1
    st.on_attempt_finished(False, 'timeout')        # wp2 skipped
    st.on_attempt_finished(True)                    # wp3 reached
    assert st.summary() == 'reached 2/3, skipped: wp2 (timeout)'

    st = PlaybackState(_wps(Waypoint, ['wp1', 'wp2'], [(0, 0), (1, 0)]), retries=0)
    st.on_attempt_finished(True)
    st.on_attempt_finished(True)
    assert st.summary() == 'reached 2/2'

    st = PlaybackState(_wps(Waypoint, ['wp1', 'wp2'], [(0, 0), (1, 0)]), retries=0)
    st.on_attempt_finished(False, 'aborted')
    st.on_attempt_finished(True)
    assert st.summary() == 'reached 1/2, skipped: wp1 (aborted)'

    # --- goal orientation helpers ---
    q = quaternion_from_yaw(0.0)
    assert q == {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
    q = quaternion_from_yaw(math.pi / 2)
    assert abs(q['z'] - math.sin(math.pi / 4)) < 1e-12
    assert abs(q['w'] - math.cos(math.pi / 4)) < 1e-12
    assert abs(yaw_to_face((0.0, 0.0), (1.0, 1.0)) - math.pi / 4) < 1e-12
    trio = _wps(Waypoint, ['a', 'b', 'c'], [(0.0, 0.0), (1.0, 1.0), (2.0, 2.0)])
    assert goal_orientation(trio, 2) == m.IDENTITY_QUATERNION  # last: identity
    g0 = goal_orientation(trio, 0)                  # face b: 45 deg
    g1 = goal_orientation(trio, 1)                  # face c: 45 deg
    for g in (g0, g1):
        assert abs(g['z'] - math.sin(math.pi / 8)) < 1e-12
        assert abs(g['w'] - math.cos(math.pi / 8)) < 1e-12

    # --- teach session (auto names, undo, dirty bookkeeping) ---
    ts = TeachSession()
    assert ts.dirty is False
    a = ts.record(1.0, 2.0, 0.0)
    assert a.name == 'wp1'
    b = ts.record(3.0, 4.0, 0.1)
    assert b.name == 'wp2'
    assert ts.dirty is True
    assert ts.undo() is b and len(ts.waypoints) == 1
    c = ts.record(5.0, 6.0, 0.0)
    assert c.name == 'wp2'                          # freed name reused
    assert ts.undo() is c
    assert ts.undo() is a
    assert ts.undo() is None
    with tempfile.TemporaryDirectory() as tmp:
        p = pathlib.Path(tmp) / 't.yaml'
        assert ts.save(p) == 0
        assert ts.dirty is False
        assert load_waypoints(p) == []

    # --- whole-module bytecode compile (imports never execute) ---
    py_compile.compile(str(SRC), doraise=True)

    print('test_nav_waypoint_teach_repeat.py: ALL PASS')


if __name__ == '__main__':
    main()
