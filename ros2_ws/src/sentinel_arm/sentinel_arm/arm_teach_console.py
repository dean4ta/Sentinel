#!/usr/bin/env python3
# Interactive arm teach-and-repeat console.
#
# Run on the robot, with the bridge already up:
#   ros2 launch sentinel_arm arm_teach.launch.py
#   ros2 run sentinel_arm arm_teach_console
#
# Keys (single char at the "arm> " prompt):
#   t / T     torque off (hand-guide) / torque on
#   r         record current pose as a waypoint (name prompt, enter = default)
#   l         list waypoints
#   d         remove a waypoint (name prompt)
#   n         rename a waypoint ("old new" prompt)
#   o         reorder a waypoint ("name index" prompt)
#   s         save trajectory (filename prompt, enter = default)
#   L         load trajectory (filename prompt)
#   p         start playback (filename prompt)
#   P / R / S pause / resume / stop playback
#   h         help
#   q         quit (ctrl+c / ctrl+d also work)
#
# In a line prompt: enter submits, backspace deletes, ctrl+u clears,
# ctrl+c cancels the entry (ctrl+c at the main prompt quits).
# arm/teach_status messages print as they arrive.

import os
import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from sentinel_msgs.srv import Playback, RecordWaypoint, SetTorque, WaypointCmd

JOINT_NAMES = ['base', 'shoulder', 'elbow', 'clamp']

HELP = """t/T   torque off/on        r   record waypoint
l     list waypoints       d   remove waypoint
n     rename ("old new")   o   reorder ("name index")
s     save trajectory      L   load trajectory
p     play trajectory      P   pause   R resume   S stop
h     help                 q   quit"""


class ArmTeachConsole(Node):

    def __init__(self):
        super().__init__('arm_teach_console')
        self.joint_pos = {}          # name -> rad, latest feedback
        self.pending_status = []     # strings queued for the terminal
        self.record_count = 0
        self._svc_clients = {}
        self.state_sub = self.create_subscription(
            JointState, 'arm/joint_states', self._state_cb, 10)
        self.status_sub = self.create_subscription(
            String, 'arm/teach_status', self._status_cb, 10)

    def _state_cb(self, msg):
        self.joint_pos = dict(zip(msg.name, msg.position))

    def _status_cb(self, msg):
        self.pending_status.append(msg.data)

    def call(self, srv_type, srv_name, request):
        cli = self._svc_clients.get(srv_name)
        if cli is None:
            cli = self.create_client(srv_type, srv_name)
            self._svc_clients[srv_name] = cli
        if not cli.wait_for_service(timeout_sec=3.0):
            return None, f'{srv_name}: service not available'
        fut = cli.call_async(request)
        while rclpy.ok() and not fut.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        return fut.result(), None

    # --- service wrappers (return user-facing message) ---

    def torque(self, enable):
        req = SetTorque.Request()
        req.enable = enable
        resp, err = self.call(SetTorque, 'arm/torque', req)
        if err:
            return err
        if resp.success:
            return 'torque ' + ('on' if enable else 'off (hand-guide)')
        return 'torque call failed (bridge not ready?)'

    def record(self, name):
        req = RecordWaypoint.Request()
        req.name = name
        resp, err = self.call(RecordWaypoint, 'arm/record_waypoint', req)
        if err:
            return err
        if resp.success:
            self.record_count += 1
        return resp.message

    def manage(self, cmd, arg=''):
        req = WaypointCmd.Request()
        req.cmd = cmd
        req.arg = arg
        resp, err = self.call(WaypointCmd, 'arm/waypoint_cmd', req)
        return err or resp.message

    def play(self, cmd, filename=''):
        req = Playback.Request()
        req.cmd = cmd
        req.filename = filename
        resp, err = self.call(Playback, 'arm/playback', req)
        return err or resp.message

    def pose_str(self):
        if not self.joint_pos or any(
                n not in self.joint_pos for n in JOINT_NAMES):
            return 'no pose yet'
        return '[%s]' % ', '.join(
            f'{self.joint_pos[n]:.2f}' for n in JOINT_NAMES)


def _read_line(label):
    """Raw-mode line editor. Returns (line, True) or (None, False) if
    cancelled. Caller keeps raw mode on throughout."""
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
        elif ch == b'\x15':           # ctrl+u
            buf = ''
        elif ch == b'\x03':           # ctrl+c: cancel entry
            sys.stdout.write('\n')
            return None, False
        elif 32 <= ch[0] < 127:
            buf += ch.decode()
        sys.stdout.write(f'\r{label}> {buf}')
        sys.stdout.flush()


def main(args=None):
    rclpy.init(args=args)
    console = ArmTeachConsole()
    old = termios.tcgetattr(sys.stdin.fileno())
    tty.setcbreak(sys.stdin.fileno())
    prompt = 'arm> '
    try:
        sys.stdout.write(f'{HELP}\n\n{console.pose_str()}\n')
        sys.stdout.write(f'\r{prompt}')
        sys.stdout.flush()
        while rclpy.ok():
            # drain playback status messages
            while console.pending_status:
                sys.stdout.write(f'\r[{console.pending_status.pop(0)}]\n')
            r, _, _ = select.select([sys.stdin], [], [], 0.05)
            rclpy.spin_once(console, timeout_sec=0.05)
            if not r:
                continue
            ch = os.read(sys.stdin.fileno(), 1)
            if ch in (b'\x03', b'\x04'):  # ctrl+c / ctrl+d: quit
                break
            if ch == b'\r':
                continue
            c = ch.decode()
            out = None
            if c == 'q':
                break
            elif c == 'h':
                out = HELP
            elif c == 't':
                out = console.torque(False)
            elif c == 'T':
                out = console.torque(True)
            elif c == 'l':
                out = 'waypoints: ' + console.manage('list')
            elif c == 'r':
                name, ok = _read_line(
                    f"record (current {console.pose_str()}) name "
                    f"[wp{console.record_count + 1:02d}]")
                if ok:
                    out = console.record(name or f'wp{console.record_count + 1:02d}')
            elif c == 'd':
                name, ok = _read_line('remove waypoint name')
                if ok and name:
                    out = console.manage('remove', name)
            elif c == 'n':
                line, ok = _read_line('rename ("old new")')
                if ok and line:
                    out = console.manage('rename', line)
            elif c == 'o':
                line, ok = _read_line('reorder ("name index")')
                if ok and line:
                    out = console.manage('reorder', line)
            elif c == 's':
                name, ok = _read_line('save as')
                if ok:
                    out = console.manage('save', name)
            elif c == 'L':
                name, ok = _read_line('load file')
                if ok and name:
                    out = console.manage('load', name)
            elif c == 'p':
                name, ok = _read_line('play file')
                if ok and name:
                    out = console.play('start', name)
            elif c == 'P':
                out = console.play('pause')
            elif c == 'R':
                out = console.play('resume')
            elif c == 'S':
                out = console.play('stop')
            else:
                out = 'unknown key, h for help'
            if out:
                sys.stdout.write(f'\r{out}\n')
            sys.stdout.write(f'\r{prompt}')
            sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old)
        sys.stdout.write('\nbye\n')
        console.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
