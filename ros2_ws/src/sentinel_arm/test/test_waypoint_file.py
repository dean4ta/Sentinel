#!/usr/bin/env python3
# Self-check for waypoint_file. No ROS, no framework.
#   python3 test_waypoint_file.py   (any env with PyYAML)

import os
import sys
import tempfile

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'sentinel_arm'))
from waypoint_file import (  # noqa: E402
    JOINT_NAMES, NUM_JOINTS, Trajectory, Waypoint, WaypointError)

FAIL = 0


def check(cond, msg):
    global FAIL
    if not cond:
        print(f'FAIL: {msg}')
        FAIL += 1


def expect_error(fn, msg):
    global FAIL
    try:
        fn()
        print(f'FAIL: {msg} (no error raised)')
        FAIL += 1
    except WaypointError:
        pass


# build
t = Trajectory(name='demo')
t.add(Waypoint('home', [0.0, 0.5, -0.3, 0.0], delay_after=0.5))
t.add(Waypoint('grasp', [0.35, 0.7, -0.6, 1.1]))
check(t.names() == ['home', 'grasp'], 'names after add')
check(t.waypoints[0].delay_after == 0.5, 'delay kept')

# rejects
expect_error(lambda: t.add(Waypoint('home', [1] * NUM_JOINTS)),
             'duplicate add accepted')
expect_error(lambda: t.add(Waypoint('bad name', [0] * NUM_JOINTS)),
             'bad name accepted')
expect_error(lambda: t.add(Waypoint('short', [0, 0, 0])),
             'short position accepted')
expect_error(lambda: t.add(Waypoint('neg', [0] * NUM_JOINTS, delay_after=-1)),
             'negative delay accepted')
expect_error(lambda: t.rename('home', 'grasp'), 'rename-to-dup accepted')
expect_error(lambda: t.remove('nope'), 'remove missing accepted')
expect_error(lambda: t.reorder('grasp', 99), 'bad reorder index accepted')

# rename + reorder
t.rename('home', 'start')
t.add(Waypoint('lift', [0.0, 1.1, -0.9, 1.1]))
t.reorder('lift', 0)
check(t.names() == ['lift', 'start', 'grasp'], 'reorder result')

# roundtrip
with tempfile.TemporaryDirectory() as d:
    path = os.path.join(d, 'demo.yaml')
    t.save_file(path)
    t2 = Trajectory.load_file(path)
    check(t2.names() == t.names(), 'roundtrip names')
    check(t2.name == 'demo', 'roundtrip trajectory name')
    check(t2.waypoints[1].position == t.waypoints[1].position,
          'roundtrip positions')
    check(t2.waypoints[1].delay_after == 0.5, 'roundtrip delay')
    check(t2.created == t.created, 'roundtrip created')

# bad file rejected
with tempfile.TemporaryDirectory() as d:
    bad = os.path.join(d, 'bad.yaml')
    with open(bad, 'w') as f:
        f.write('name: x\njoint_names: [a, b, c, d]\nwaypoints: []\n')
    expect_error(lambda: Trajectory.load_file(bad), 'bad header accepted')

# remove + clear
t.remove('grasp')
check(t.names() == ['lift', 'start'], 'remove result')
t.clear()
check(t.names() == [], 'clear result')

# ignore list
t.add(Waypoint('hold', [0.0] * NUM_JOINTS, ignore=['clamp']))
check(t.waypoints[0].ignore == ['clamp'], 'ignore kept')
expect_error(lambda: t.add(Waypoint('badj', [0] * NUM_JOINTS,
                                    ignore=['nose'])),
             'unknown ignore joint accepted')
expect_error(lambda: t.add(Waypoint('dupj', [0] * NUM_JOINTS,
                                    ignore=['clamp', 'clamp'])),
             'duplicate ignore joint accepted')
with tempfile.TemporaryDirectory() as d:
    path = os.path.join(d, 'ig.yaml')
    t.save_file(path)
    t3 = Trajectory.load_file(path)
    check(t3.waypoints[0].ignore == ['clamp'], 'ignore roundtrip')

check(JOINT_NAMES == ['base', 'shoulder', 'elbow', 'clamp'], 'JOINT_NAMES')

if FAIL:
    print(f'{FAIL} check(s) failed')
    sys.exit(1)
print('all checks passed')
