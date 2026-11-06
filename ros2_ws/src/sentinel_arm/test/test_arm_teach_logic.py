#!/usr/bin/env python3
# Playback state-machine self-check for arm_teach, without ROS.
# Stubs rclpy/sensor_msgs/std_msgs/sentinel_msgs so arm_teach.py imports
# in a plain Python env, then drives the node's callbacks directly.
#
#   python3 test_arm_teach_logic.py   (stdlib only, no PyYAML/ROS needed)

import os
import sys
import tempfile
import time
import types

HERE = os.path.dirname(os.path.abspath(__file__))
# parent of the sentinel_arm package dir, so package imports resolve
sys.path.insert(0, os.path.join(HERE, '..'))

params = {}  # declare_parameter stores here; get_parameter reads it


class _FakeClock:
    def now(self):
        return _FakeTime()


class _FakeTime:
    def to_msg(self):
        return None


class _FakeLogger:
    def info(self, *a):
        pass

    def warn(self, *a):
        pass


class _Publisher:
    def __init__(self):
        self.msgs = []

    def publish(self, m):
        self.msgs.append(m)


class _NodeStub:
    def __init__(self, *a, **kw):
        pass

    def declare_parameter(self, name, default):
        params[name] = default

    def get_parameter(self, name):
        class R:
            value = params[name]
        return R()

    def get_clock(self):
        return _FakeClock()

    def get_logger(self):
        return _FakeLogger()

    def create_subscription(self, *a, **kw):
        return None

    def create_publisher(self, *a, **kw):
        return _Publisher()

    def create_timer(self, *a, **kw):
        return None

    def create_service(self, *a, **kw):
        return None

    def destroy_node(self):
        pass


def _mod(name):
    m = types.ModuleType(name)
    sys.modules[name] = m
    return m


rclpy = _mod('rclpy')
rclpy.ok = lambda: True
rclpy.init = lambda *a, **kw: None
rclpy.spin = lambda n: None
rclpy.shutdown = lambda: None
rclpy.node = _mod('rclpy.node')
rclpy.node.Node = _NodeStub

sensor_msgs = _mod('sensor_msgs')
sensor_msgs.msg = _mod('sensor_msgs.msg')


class _Header:
    stamp = None


class _JointState:
    def __init__(self):
        self.header = _Header()
        self.name = []
        self.position = []


sensor_msgs.msg.JointState = _JointState

std_msgs = _mod('std_msgs')
std_msgs.msg = _mod('std_msgs.msg')
std_msgs.msg.String = type('String', (), {})

sentinel_msgs = _mod('sentinel_msgs')
sentinel_msgs.srv = _mod('sentinel_msgs.srv')
for _n in ('Playback', 'RecordWaypoint', 'WaypointCmd'):
    setattr(sentinel_msgs.srv, _n, type(_n, (), {}))

import sentinel_arm.arm_teach as arm_teach  # noqa: E402  (stubs first)
from sentinel_arm.waypoint_file import (  # noqa: E402
    JOINT_NAMES, Trajectory, Waypoint)

FAIL = 0


def check(cond, msg):
    global FAIL
    if not cond:
        print(f'FAIL: {msg}')
        FAIL += 1


def jstate(pos):
    m = sensor_msgs.msg.JointState()
    m.name = list(JOINT_NAMES)
    m.position = list(pos)
    return m


class Req:
    pass


class Resp:
    pass


def svc(fn, **kw):
    req = Req()
    for k, v in kw.items():
        setattr(req, k, v)
    return fn(req, Resp())


def statuses(n):
    return [m.data for m in n.status_pub.msgs]


with tempfile.TemporaryDirectory() as d:
    n = arm_teach.ArmTeach()
    n.waypoint_dir = d

    # --- record two poses through the service ---
    n._state_cb(jstate([0.0, 0.0, 0.0, 0.0]))
    r = svc(n._record_cb, name='p1')
    check(getattr(r, 'success', False) and len(n.traj.waypoints) == 1, 'record p1')
    n._state_cb(jstate([0.1, 0.2, -0.3, 0.4]))
    r = svc(n._record_cb, name='p2')
    check(getattr(r, 'success', False) and len(n.traj.waypoints) == 2, 'record p2')
    r = svc(n._record_cb, name='p1')
    check(not getattr(r, 'success', False), 'duplicate record rejected')
    # no feedback yet
    n._joint_pos = {}
    r = svc(n._record_cb, name='p3')
    check(not getattr(r, 'success', False), 'record without feedback rejected')

    # p2 gets a delay so the holding path runs; set before save so the
    # file (which 'start' reloads) carries it
    n.traj.waypoints[1].delay_after = 0.05

    # --- save/load through the service ---
    r = svc(n._manage_cb, cmd='save', arg='demo')
    check(getattr(r, 'success', False) and os.path.exists(os.path.join(d, 'demo.yaml')),
          'save writes file')
    r = svc(n._manage_cb, cmd='clear', arg='')
    check(getattr(r, 'success', False) and n.traj.names() == [], 'clear')
    r = svc(n._manage_cb, cmd='load', arg='demo')
    check(getattr(r, 'success', False) and n.traj.names() == ['p1', 'p2'], 'load restores')
    r = svc(n._manage_cb, cmd='rename', arg='p1 first')
    check(getattr(r, 'success', False) and n.traj.names() == ['first', 'p2'],
          'rename via service')
    n.traj.rename('first', 'p1')

    # --- playback happy path: p1 -> p2 -> hold -> done ---
    r = svc(n._play_cb, cmd='start', filename='demo')
    check(getattr(r, 'success', False) and n.play_state == 'playing', 'start loads and plays')
    n._tick()  # sends p1
    check(n.goal == [0.0, 0.0, 0.0, 0.0], 'goal p1 sent')
    n._state_cb(jstate([0.0, 0.0, 0.0, 0.0]))
    n._tick()  # hold 1
    n._tick()  # hold 2 -> advance past p1 (no delay)
    check(n.play_state == 'playing' and n.goal is None, 'advanced past p1')
    n._tick()  # sends p2
    check(n.goal == [0.1, 0.2, -0.3, 0.4], 'goal p2 sent')
    n._state_cb(jstate([0.1, 0.2, -0.3, 0.4]))
    n._tick()  # hold 1
    n._tick()  # hold 2 -> p2 has delay -> holding
    check(n.play_state == 'holding', 'enters holding after p2')
    check(any('holding at p2' in s for s in statuses(n)), 'holding status')
    n._tick()  # wait_until not reached yet
    check(n.play_state == 'holding', 'still holding before delay elapses')
    time.sleep(0.06)
    n._tick()  # delay done -> done
    check(n.play_state == 'idle' and 'done' in statuses(n),
          'playback completes')
    check([list(m.position) for m in n.goal_pub.msgs][:2] ==
          [[0.0, 0.0, 0.0, 0.0], [0.1, 0.2, -0.3, 0.4]],
          'goals published in order')

    # --- timeout aborts ---
    n.timeout = 0.05
    n.play_state = 'playing'
    n.play_index = 0
    n.goal = None
    n._tick()  # sends p1 again
    n._state_cb(jstate([9.0, 9.0, 9.0, 9.0]))  # never converges
    time.sleep(0.06)
    n._tick()
    check(n.play_state == 'idle', 'timeout aborts playback')
    check(any('not reached' in s for s in statuses(n)), 'timeout status')

    # --- pause / resume / stop ---
    r = svc(n._play_cb, cmd='start', filename='demo')
    check(getattr(r, 'success', False), 'restart for pause test')
    n._tick()  # sends p1
    n._state_cb(jstate([0.0, 0.0, 0.0, 0.0]))
    n._tick()  # hold 1
    r = svc(n._play_cb, cmd='pause', filename='')
    check(getattr(r, 'success', False) and n.play_state == 'paused', 'pause')
    r = svc(n._play_cb, cmd='resume', filename='')
    check(getattr(r, 'success', False) and n.play_state == 'playing', 'resume')
    r = svc(n._play_cb, cmd='stop', filename='')
    check(getattr(r, 'success', False) and n.play_state == 'idle', 'stop')
    r = svc(n._play_cb, cmd='stop', filename='')
    check(not getattr(r, 'success', False), 'stop while idle rejected')

if FAIL:
    print(f'{FAIL} check(s) failed')
    sys.exit(1)
print('all checks passed')
