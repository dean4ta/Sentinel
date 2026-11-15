#!/usr/bin/env python3
"""T012 selftest: step_recorder + controller_level_analysis pure logic
(no ROS, stdlib only).

T009/T011 pattern: both modules under test import their heavy deps at a
'wiring' seam (step_recorder: `import rclpy`; the Mac analysis script:
`import numpy`), so this test never executes a full module. It:

1. AST-extracts the pure sections (py310 grammar check by construction).
2. Exercises CSV row formatting / field order / yaw math / odom-fallback
   selection / step-event parsing / level-dir naming with plain asserts,
   including a recorder-write -> analysis-parse round trip.
3. AST-checks the launch file (controller_level arg, node set, EKF
   remap, LEVEL_PARAMS mapping).
4. py_compile-compiles all changed files (bytecode only, no imports).

Run (Mac, no ROS):
  python3 test/test_step_recorder.py
"""

import ast
import math
import pathlib
import py_compile
import tempfile
import types

HERE = pathlib.Path(__file__).resolve().parent
SRC_REC = HERE.parent / 'sentinel_debug' / 'step_recorder.py'
SRC_ANALYSIS = HERE.parents[3] / 'utils' / 'controller_level_analysis.py'
SRC_LAUNCH = HERE.parents[1] / 'sentinel_drive_cpp' / 'launch' / \
    'step_test.launch.py'


def load_pure(src, seam):
    """Compile and exec everything above the first `import <seam>`."""
    source = src.read_text()
    tree = ast.parse(source, filename=str(src), feature_version=(3, 10))
    idx = next(
        i for i, n in enumerate(tree.body)
        if isinstance(n, ast.Import) and any(a.name == seam for a in n.names))
    assert idx > 0, f'no pure section above the {seam} import seam'
    pure = ast.fix_missing_locations(
        ast.Module(body=tree.body[:idx], type_ignores=[]))
    mod = types.ModuleType('_pure_' + src.stem)
    code = compile(pure, str(src), 'exec')
    exec(code, mod.__dict__)
    return mod


def _quat(x, y, z, w):
    """Stand-in for a geometry_msgs Quaternion."""
    return types.SimpleNamespace(x=x, y=y, z=z, w=w)


def _odom(x, y, yaw, vx, wz):
    """Stand-in for a nav_msgs/Odometry (only the fields odom_row
    reads)."""
    qz = math.sin(yaw / 2)
    qw = math.cos(yaw / 2)
    return types.SimpleNamespace(
        header=types.SimpleNamespace(
            stamp=types.SimpleNamespace(sec=10, nanosec=0)),
        pose=types.SimpleNamespace(
            pose=types.SimpleNamespace(
                position=types.SimpleNamespace(x=x, y=y),
                orientation=_quat(0.0, 0.0, qz, qw))),
        twist=types.SimpleNamespace(
            twist=types.SimpleNamespace(
                linear=types.SimpleNamespace(x=vx),
                angular=types.SimpleNamespace(z=wz))))


def launch_defaults():
    """{arg_name: default} from the DeclareLaunchArgument calls."""
    tree = ast.parse(SRC_LAUNCH.read_text(), filename=str(SRC_LAUNCH),
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


def launch_nodes():
    """{(package, executable)} from every Node(...) call in the launch
    file, plus the ekf node's remappings."""
    tree = ast.parse(SRC_LAUNCH.read_text(), filename=str(SRC_LAUNCH),
                     feature_version=(3, 10))
    pairs = set()
    remaps = []
    for node in ast.walk(tree):
        if (isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
                and node.func.id == 'Node'):
            kw = {k.arg: k.value for k in node.keywords}
            if 'package' in kw and 'executable' in kw:
                pairs.add((kw['package'].value, kw['executable'].value))
            if 'remappings' in kw:
                remaps.append(ast.literal_eval(kw['remappings']))
    return pairs, remaps


def main():
    rec = load_pure(SRC_REC, 'rclpy')
    ana = load_pure(SRC_ANALYSIS, 'numpy')

    # --- robot/Mac CSV contract: identical columns + filenames ---
    expected_fields = (
        'fl_target_rpm', 'fr_target_rpm', 'bl_target_rpm', 'br_target_rpm',
        'fl_measured_rpm_filtered', 'fr_measured_rpm_filtered',
        'bl_measured_rpm_filtered', 'br_measured_rpm_filtered',
        'fl_measured_rpm_raw', 'fr_measured_rpm_raw',
        'bl_measured_rpm_raw', 'br_measured_rpm_raw',
        'fl_cmd_rpm', 'fr_cmd_rpm', 'bl_cmd_rpm', 'br_cmd_rpm')
    assert rec.WHEEL_SPEEDS_FIELDS == expected_fields
    assert ana.WHEEL_SPEEDS_FIELDS == rec.WHEEL_SPEEDS_FIELDS
    assert ana.ODOM_FIELDS == rec.ODOM_FIELDS == ('x', 'y', 'yaw', 'vx', 'wz')
    assert ana.STEP_EVENT_FIELDS == rec.STEP_EVENT_FIELDS == \
        ('index', 'target_mps', 'sine')
    for const in ('WHEEL_SPEEDS_CSV', 'WHEEL_ODOM_CSV', 'FILTERED_ODOM_CSV',
                  'STEP_EVENTS_CSV'):
        assert getattr(ana, const) == getattr(rec, const)

    # --- stamp -> float seconds ---
    assert rec.stamp_float(1234, 567890000) == 1234.56789
    assert rec.stamp_float(5, 0) == 5.0
    assert isinstance(rec.stamp_float(0, 1), float)
    stamped = types.SimpleNamespace(
        header=types.SimpleNamespace(
            stamp=types.SimpleNamespace(sec=7, nanosec=250000000)))
    assert rec.header_stamp_float(stamped) == 7.25

    # --- row formatting: field order per msg, plain numeric CSV ---
    hdr = rec.make_header(rec.WHEEL_SPEEDS_FIELDS)
    assert hdr == 't,' + ','.join(expected_fields)
    assert rec.make_header(rec.ODOM_FIELDS) == 't,x,y,yaw,vx,wz'
    assert rec.format_row([1.5, 2.0]) == '1.5,2.0'
    assert rec.format_row([0, -3]) == '0,-3'
    msg = types.SimpleNamespace(**{f: 0.0 for f in rec.WHEEL_SPEEDS_FIELDS})
    msg.fl_target_rpm = 3.5
    msg.br_cmd_rpm = -2.25
    row = rec.wheel_speeds_row(msg, 9.0)
    assert len(row) == 1 + len(expected_fields)
    assert row[0] == 9.0 and row[1] == 3.5 and row[-1] == -2.25
    assert row[5] == 0.0  # fl_measured_rpm_filtered sits 5th in the msg

    # --- yaw from quaternion (manual atan2) ---
    assert rec.yaw_from_quat(_quat(0.0, 0.0, 0.0, 1.0)) == 0.0
    for yaw in (0.0, 0.7, -1.2, math.pi / 2, -math.pi / 2):
        qz = math.sin(yaw / 2)
        qw = math.cos(yaw / 2)
        assert math.isclose(
            rec.yaw_from_quat(_quat(0.0, 0.0, qz, qw)), yaw)
    assert math.isclose(
        rec.yaw_from_quat(_quat(0.0, 0.0, 1.0, 0.0)), math.pi, abs_tol=1e-9)

    # --- odometry row: [t, x, y, yaw, vx, wz] ---
    row = rec.odom_row(_odom(1.0, 2.0, 0.5, 0.3, -0.1), 7.5)
    assert len(row) == 6
    assert row[0] == 7.5 and row[1] == 1.0 and row[2] == 2.0
    assert math.isclose(row[3], 0.5) and row[4] == 0.3 and row[5] == -0.1

    # --- step events: bool sine -> 0/1, fields in msg order ---
    ev = types.SimpleNamespace(index=2, target_mps=0.5, sine=True)
    assert rec.step_event_row(ev, 9.0) == [9.0, 2, 0.5, 1]
    ev.sine = False
    assert rec.step_event_row(ev, 9.0) == [9.0, 2, 0.5, 0]

    # --- level -> dir naming (recorder side + analysis acceptance) ---
    assert rec.default_out_dir('off') == 'data/controller_level_off'
    assert rec.default_out_dir('ffpid') == 'data/controller_level_ffpid'
    with tempfile.TemporaryDirectory() as tmp:
        tmp = pathlib.Path(tmp)
        (tmp / 'level_ff').mkdir()
        assert ana.find_level_dir(tmp, 'ff') == tmp / 'level_ff'
        assert ana.find_level_dir(tmp, 'off') is None
        (tmp / 'controller_level_off').mkdir()
        assert ana.find_level_dir(tmp, 'off') == \
            tmp / 'controller_level_off'

        # --- odom fallback: filtered present -> filtered; else wheel ---
        ldir = tmp / 'level_ffpid'
        ldir.mkdir()
        assert ana.choose_odom_csv(ldir) is None
        wheel = ldir / 'wheel_odom.csv'
        wheel.write_text('t,x,y,yaw,vx,wz\n')
        assert ana.choose_odom_csv(ldir) == wheel
        filtered = ldir / 'wheel_imu_odom_filtered.csv'
        filtered.write_text('t,x,y,yaw,vx,wz\n')
        assert ana.choose_odom_csv(ldir) == filtered
        assert 'wheel_imu_odom_filtered' in ana.odom_source_label(filtered)
        assert 'wheel_odom' in ana.odom_source_label(wheel)
        assert 'EKF' not in ana.odom_source_label(wheel)

        # --- recorder-write -> analysis-parse round trip ---
        speeds = rec.CsvSink(str(ldir / 'wheel_speeds.csv'),
                             rec.WHEEL_SPEEDS_FIELDS)
        speeds.append(rec.wheel_speeds_row(msg, 1.0))
        msg.fl_target_rpm = -4.0
        speeds.append(rec.wheel_speeds_row(msg, 2.0))
        speeds.flush()
        speeds.close()
        data = ana.parse_csv(ldir / 'wheel_speeds.csv')
        assert data['t'] == [1.0, 2.0]
        assert data['fl_target_rpm'] == [3.5, -4.0]
        assert data['br_cmd_rpm'] == [-2.25, -2.25]
        series = ana.wheel_series(data)
        for wheel in ana.WHEELS:
            assert series[wheel + '_target'] == \
                data[wheel + '_target_rpm']
            assert series[wheel + '_measured'] == \
                data[wheel + '_measured_rpm_filtered']

        # --- step-event parsing ---
        events = rec.CsvSink(str(ldir / 'step_events.csv'),
                             rec.STEP_EVENT_FIELDS)
        events.append(rec.step_event_row(ev, 3.0))
        events.close()
        steps = ana.parse_csv(ldir / 'step_events.csv')
        assert steps['t'] == [3.0]
        assert steps['index'] == [2.0] and steps['target_mps'] == [0.5]
        assert steps['sine'] == [0.0]

    # --- launch file: controller_level arg + node set + EKF remap ---
    defaults = launch_defaults()
    assert defaults.get('controller_level') == 'ffpid'
    tree = ast.parse(SRC_LAUNCH.read_text(), filename=str(SRC_LAUNCH),
                     feature_version=(3, 10))
    choices = None
    level_params = None
    for node in ast.walk(tree):
        if (isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
                and node.func.id == 'DeclareLaunchArgument'):
            if node.args and node.args[0].value == 'controller_level':
                choices = ast.literal_eval(
                    next(kw.value for kw in node.keywords
                         if kw.arg == 'choices'))
        if (isinstance(node, ast.Assign) and
                any(isinstance(t, ast.Name) and t.id == 'LEVEL_PARAMS'
                    for t in node.targets)):
            level_params = ast.literal_eval(node.value)
    assert choices == ['off', 'ff', 'ffpid']
    assert level_params == {
        'off': {'pid_enabled': False, 'feed_forward_enabled': False},
        'ff': {'pid_enabled': False, 'feed_forward_enabled': True},
        'ffpid': {'pid_enabled': True, 'feed_forward_enabled': True},
    }
    pairs, remaps = launch_nodes()
    for pair in (('sentinel_drive_cpp', 'cmd_vel_serial'),
                 ('sentinel_drive_cpp', 'wheel_odometry'),
                 ('sentinel_drive_cpp', 'wheel_slip_detector'),
                 ('robot_localization', 'ekf_node'),
                 ('sentinel_debug', 'step_recorder')):
        assert pair in pairs, f'launch missing {pair}'
    assert any(('odometry/filtered', '/wheel_imu_odom_filtered') in r
               for r in remaps), 'EKF output remap missing'
    assert 'data/controller_level_' in SRC_LAUNCH.read_text()

    # --- py310 grammar + bytecode: all changed files ---
    for src in (SRC_REC, SRC_ANALYSIS, SRC_LAUNCH, pathlib.Path(__file__)):
        ast.parse(src.read_text(), filename=str(src),
                  feature_version=(3, 10))
        py_compile.compile(str(src), doraise=True)

    print('test_step_recorder.py: ALL PASS')


if __name__ == '__main__':
    main()
