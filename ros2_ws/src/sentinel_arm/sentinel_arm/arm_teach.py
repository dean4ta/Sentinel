#!/usr/bin/env python3
# Arm teach-and-repeat node.
#
# No serial access — poses come from the bridge's arm/joint_states,
# motion goes out through the bridge's arm/joint_goals. Opening the
# serial port here would reset the arm's ESP32 and wedge the bridge,
# so this node must never import serial or open /dev/sentinel_arm.
#
# Services:
#   arm/record_waypoint  RecordWaypoint  - snapshot current pose
#   arm/waypoint_cmd     WaypointCmd     - list/remove/rename/reorder/
#                                          clear/save/load
#   arm/playback         Playback        - start/pause/resume/stop
# Topics:
#   arm/teach_status     std_msgs/String - playback state machine
#
# Playback: send a waypoint's joints, wait until arm/joint_states is
# within reach_tol for hold_samples consecutive polls (bridge polls at
# ~10 Hz; this node checks on its own 10 Hz timer), then next. Timeout
# freezes the arm and aborts. Pause/stop command the current joints
# once (hold); resume re-sends the paused waypoint.

import os
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from sentinel_msgs.srv import Playback, RecordWaypoint, WaypointCmd
from sentinel_arm.waypoint_file import (JOINT_NAMES, Trajectory, Waypoint,
                                        WaypointError)


class ArmTeach(Node):

    def __init__(self):
        super().__init__('arm_teach')
        self.declare_parameter('waypoint_dir', '~/arm_waypoints')
        self.declare_parameter('reach_tol', 0.02)        # rad
        self.declare_parameter('hold_samples', 2)        # consecutive polls
        self.declare_parameter('waypoint_timeout', 30.0)  # s per waypoint

        self.waypoint_dir = os.path.expanduser(
            self.get_parameter('waypoint_dir').value)
        self.reach_tol = self.get_parameter('reach_tol').value
        self.hold_samples = self.get_parameter('hold_samples').value
        self.timeout = self.get_parameter('waypoint_timeout').value

        self.traj = Trajectory()
        self._joint_pos = {}      # name -> rad, latest feedback
        self._last_state = None   # raw JointState, for hold/freeze
        self.play_state = 'idle'  # idle|playing|paused|holding
        self.play_index = 0
        self.goal = None          # [rad]*4 being awaited
        self.goal_hold = 0
        self.goal_deadline = 0.0
        self.wait_until = 0.0

        self.state_sub = self.create_subscription(
            JointState, 'arm/joint_states', self._state_cb, 10)
        self.goal_pub = self.create_publisher(JointState, 'arm/joint_goals', 10)
        self.status_pub = self.create_publisher(String, 'arm/teach_status', 10)
        self.create_service(RecordWaypoint, 'arm/record_waypoint',
                            self._record_cb)
        self.create_service(WaypointCmd, 'arm/waypoint_cmd', self._manage_cb)
        self.create_service(Playback, 'arm/playback', self._play_cb)
        self._tick_timer = self.create_timer(0.1, self._tick)
        self.get_logger().info(
            f'arm_teach ready, waypoints in {self.waypoint_dir}')

    # --- feedback / motion ---

    def _state_cb(self, msg):
        self._last_state = msg
        self._joint_pos = dict(zip(msg.name, msg.position))

    def _send_goal(self, position):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = [float(x) for x in position]
        self.goal_pub.publish(msg)

    def _freeze(self):
        # Command the current pose once so the arm holds in place.
        if self._last_state is not None:
            self.goal_pub.publish(self._last_state)

    def _set_status(self, text):
        self.get_logger().info(f'playback: {text}')
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    # --- playback state machine (10 Hz tick) ---

    def _reached(self):
        if not self._joint_pos or self.goal is None:
            return False
        ignore = self.traj.waypoints[self.play_index].ignore
        for name, target in zip(JOINT_NAMES, self.goal):
            if name in ignore:
                continue
            if name not in self._joint_pos:
                return False
            if abs(self._joint_pos[name] - target) > self.reach_tol:
                return False
        return True

    def _advance(self):
        wp = self.traj.waypoints[self.play_index]
        if self.play_state == 'playing' and wp.delay_after > 0:
            self.play_state = 'holding'
            self.wait_until = time.time() + wp.delay_after
            self._set_status(f'holding at {wp.name} ({wp.delay_after}s)')
            return
        self.goal = None
        self.play_index += 1
        if self.play_index >= len(self.traj.waypoints):
            self.play_state = 'idle'
            self._set_status('done')
        else:
            self.play_state = 'playing'

    def _tick(self):
        if self.play_state == 'playing':
            if self.goal is None:
                wp = self.traj.waypoints[self.play_index]
                self.goal = list(wp.position)
                self.goal_hold = 0
                self.goal_deadline = time.time() + self.timeout
                self._set_status(
                    f'playing {self.play_index + 1}/'
                    f'{len(self.traj.waypoints)}: {wp.name}')
                self._send_goal(self.goal)
            elif self._reached():
                self.goal_hold += 1
                if self.goal_hold >= self.hold_samples:
                    self._advance()
            else:
                self.goal_hold = 0
                if time.time() > self.goal_deadline:
                    # Per-joint error (current - target) in the abort
                    # message so a stuck joint is named directly.
                    parts = []
                    if self._joint_pos and self.goal is not None:
                        for name, target in zip(JOINT_NAMES, self.goal):
                            cur = self._joint_pos.get(name)
                            parts.append(f'{name} ?' if cur is None
                                         else f'{name} {cur - target:+.3f}')
                    detail = ' (' + ' '.join(parts) + ')' if parts else ''
                    wp_name = self.traj.waypoints[self.play_index].name
                    self._freeze()
                    self.goal = None
                    self.play_state = 'idle'
                    self._set_status(
                        f'error: {wp_name} not reached in '
                        f'{self.timeout}s{detail}')
        elif self.play_state == 'holding' and time.time() >= self.wait_until:
            self._advance()

    # --- services ---

    def _record_cb(self, req, resp):
        if not self._joint_pos or any(
                n not in self._joint_pos for n in JOINT_NAMES):
            resp.message = 'no joint states yet (arm bridge not ready?)'
            return resp
        position = [self._joint_pos[n] for n in JOINT_NAMES]
        try:
            self.traj.add(Waypoint(req.name.strip(), position))
        except WaypointError as e:
            resp.message = str(e)
            return resp
        resp.success = True
        resp.message = (f'recorded {req.name.strip()}: '
                        f'{[round(p, 3) for p in position]}')
        return resp

    def _path_for(self, filename):
        if not filename.endswith('.yaml'):
            filename += '.yaml'
        return os.path.join(self.waypoint_dir, filename)

    def _manage_cb(self, req, resp):
        cmd = req.cmd.strip().lower()
        arg = req.arg.strip()
        try:
            if cmd == 'list':
                resp.message = ', '.join(self.traj.names()) or '(empty)'
            elif cmd == 'remove':
                self.traj.remove(arg)
                resp.message = f'removed {arg}'
            elif cmd == 'rename':
                parts = arg.split()
                if len(parts) != 2:
                    resp.message = 'rename needs "<old> <new>"'
                    return resp
                self.traj.rename(*parts)
                resp.message = f'renamed {parts[0]} -> {parts[1]}'
            elif cmd == 'reorder':
                parts = arg.split()
                if len(parts) != 2:
                    resp.message = 'reorder needs "<name> <index>"'
                    return resp
                self.traj.reorder(parts[0], parts[1])
                resp.message = f'reordered {parts[0]} to index {parts[1]}'
            elif cmd == 'clear':
                n = len(self.traj.waypoints)
                self.traj.clear()
                resp.message = f'cleared {n} waypoints'
            elif cmd == 'save':
                filename = arg or self.traj.name
                path = self._path_for(filename)
                os.makedirs(self.waypoint_dir, exist_ok=True)
                self.traj.save_file(path)
                resp.message = (f'saved {len(self.traj.waypoints)} '
                                f'waypoints to {path}')
            elif cmd == 'load':
                path = self._path_for(arg)
                self.traj = Trajectory.load_file(path)
                resp.message = (f'loaded {len(self.traj.waypoints)} '
                                f'waypoints from {path}')
            else:
                resp.message = f'unknown cmd {req.cmd!r}'
                return resp
            resp.success = True
        except (WaypointError, OSError) as e:
            resp.message = str(e)
        return resp

    def _play_cb(self, req, resp):
        cmd = req.cmd.strip().lower()
        if cmd == 'start':
            if self.play_state != 'idle':
                resp.message = f'cannot start: state {self.play_state}'
                return resp
            filename = req.filename.strip()
            if not filename:
                resp.message = 'start requires filename'
                return resp
            try:
                self.traj = Trajectory.load_file(self._path_for(filename))
            except (WaypointError, OSError) as e:
                resp.message = f'load failed: {e}'
                return resp
            if not self.traj.waypoints:
                resp.message = 'no waypoints in file'
                return resp
            self.play_index = 0
            self.goal = None
            self.play_state = 'playing'
            resp.success = True
            resp.message = f'playing {len(self.traj.waypoints)} waypoints'
        elif cmd == 'pause':
            if self.play_state not in ('playing', 'holding'):
                resp.message = f'cannot pause: state {self.play_state}'
                return resp
            self._freeze()
            self.goal = None
            self.play_state = 'paused'
            resp.success = True
            resp.message = f'paused at waypoint {self.play_index + 1}'
            self._set_status(
                f'paused at {self.play_index + 1}/{len(self.traj.waypoints)}')
        elif cmd == 'resume':
            if self.play_state != 'paused':
                resp.message = f'cannot resume: state {self.play_state}'
                return resp
            self.play_state = 'playing'
            resp.success = True
            resp.message = f'resuming at waypoint {self.play_index + 1}'
            self._set_status('resuming')
        elif cmd == 'stop':
            if self.play_state not in ('playing', 'paused', 'holding'):
                resp.message = f'cannot stop: state {self.play_state}'
                return resp
            self._freeze()
            self.goal = None
            self.play_state = 'idle'
            resp.success = True
            resp.message = 'stopped'
            self._set_status('stopped')
        else:
            resp.message = f'unknown cmd {req.cmd!r} (start/pause/resume/stop)'
        return resp


def main(args=None):
    rclpy.init(args=args)
    node = ArmTeach()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
