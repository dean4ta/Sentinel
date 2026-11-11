#!/usr/bin/env python3
# RoArm-M2-S serial <-> ROS bridge. One node: publishes arm/joint_states
# (from T:105 pose reads) and arm/ee_pose (T:1051 x/y/z in mm plus yaw
# quaternion), and accepts arm/joint_goals (JointState -> per-joint
# T:101 move commands).
#
# arm/ee_goals (PoseStamped) -> T:1041 XYZ direct moves in
# end-effector space (see _ee_goal_cb).
#
# Opening the serial port resets the ESP32 (DTR/RTS assert on open), so
# the boot/init chatter (~8 s, arm moves to init pose) is drained before
# any command goes out. Joints: base=1 shoulder=2 elbow=3 clamp(EOAT)=4.

import json
import math
import threading
import time

import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from sentinel_msgs.srv import SetTorque

JOINT_IDS = {'base': 1, 'shoulder': 2, 'elbow': 3, 'clamp': 4}
JOINT_NAMES = ['base', 'shoulder', 'elbow', 'clamp']


class ArmBridge(Node):

    def __init__(self):
        super().__init__('arm_bridge')
        self.declare_parameter('port', '/dev/sentinel_arm')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('read_rate', 10.0)
        self.declare_parameter('spd', 200)  # T:101 steps/s
        self.declare_parameter('acc', 40)   # T:101 acc units
        # [x,y,z] mm + [t] rad, moved to after the boot drain via
        # T:1041 (XYZ direct). [0,0,0,0] = no home move. Must stay
        # non-empty: an empty list default would infer BYTE_ARRAY in
        # rclpy and reject double-array overrides.
        self.declare_parameter('home', [0.0, 0.0, 0.0, 0.0])
        # T:1041 moves from arm/ee_goals (x/y/z mm + yaw t rad). The
        # repo docs do not document T:1041 spd/acc units — assumed to
        # be the same scale as the T:101 spd/acc and the existing
        # T:1041 home move below. ee_spd defaults to 100 (slower than
        # the home move's 200) for the first autonomous runs; tune via
        # launch params.
        self.declare_parameter('ee_spd', 100)
        self.declare_parameter('ee_acc', 40)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        self.ser = serial.Serial(port, baud, timeout=0.5)
        self.ser.dtr = False
        self.ser.rts = False
        self.write_lock = threading.Lock()
        self.ready = False  # arm answered a pose read (boot done)
        self._last_not_ready_warn = 0.0  # rate limit for ee_goals warns

        # RX thread: drains the boot sequence, then parses feedback.
        self.rx_thread = threading.Thread(target=self._rx_loop,
                                          daemon=True)
        self.rx_thread.start()

        self.state_pub = self.create_publisher(
            JointState, 'arm/joint_states', 10)
        self.ee_pub = self.create_publisher(
            PoseStamped, 'arm/ee_pose', 10)
        self.goal_sub = self.create_subscription(
            JointState, 'arm/joint_goals', self._goal_cb, 10)
        self.ee_goal_sub = self.create_subscription(
            PoseStamped, 'arm/ee_goals', self._ee_goal_cb, 10)
        # arm/ee_goals throttle: a publisher at 10+ Hz would saturate
        # the serial link, starve the T:105 polls, and leave queued
        # T:1041 lines moving the arm after the publisher stops.
        # Goals are coalesced to the LATEST one and flushed by this
        # timer at most once per 100 ms.
        self._pending_ee_goal = None
        self._ee_goal_timer = self.create_timer(0.1, self._ee_goal_flush)
        self.torque_srv = self.create_service(
            SetTorque, 'arm/torque', self._torque_cb)
        rate = self.get_parameter('read_rate').value
        self.timer = self.create_timer(1.0 / rate, self._poll_timer)
        self.get_logger().info(f'Arm bridge on {port} @ {baud}')

    # --- serial ---

    def _rx_loop(self):
        # Drain reset/init chatter until the arm goes quiet.
        deadline = time.time() + 8.0
        while time.time() < deadline:
            self.ser.read(4096)
        self.ser.reset_input_buffer()
        # Handshake: ask for a pose until the arm answers (init
        # sequence may still be settling at the 8 s mark). Commands
        # sent earlier can wedge it, so nothing else writes yet.
        deadline = time.time() + 15.0
        while time.time() < deadline and not self.ready:
            self.ser.write(b'{"T":105}\n')
            line = self.ser.readline()
            if line:
                try:
                    d = json.loads(line.decode(errors='replace').strip())
                except ValueError:
                    continue
                if d.get('T') == 1051:
                    self.ready = True
                    self._handle_line(line.decode(errors='replace').strip())
            time.sleep(0.2)
        if not self.ready:
            self.get_logger().warn('no pose feedback after boot drain')
        # Optional startup move to the home pose (T:1041 XYZ direct).
        home = self.get_parameter('home').value
        if home and len(home) == 4 and any(home) and self.ready:
            spd = self.get_parameter('spd').value
            acc = self.get_parameter('acc').value
            self._write(json.dumps(
                {'T': 1041, 'x': home[0], 'y': home[1], 'z': home[2],
                 't': home[3], 'spd': spd, 'acc': acc}))
            self.get_logger().info(
                f'moving home: x={home[0]:.1f} y={home[1]:.1f} '
                f'z={home[2]:.1f} t={home[3]:.3f}')
        while rclpy.ok():
            line = self.ser.readline()
            if line:
                self._handle_line(line.decode(errors='replace').strip())

    def _write(self, data: str):
        with self.write_lock:
            self.ser.write((data + '\n').encode())

    def _handle_line(self, line):
        # Feedback lines look like {"T":1051,"x":..,"b":..,"s":..,...}
        try:
            d = json.loads(line)
        except (ValueError, TypeError):
            return
        if d.get('T') != 1051:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = [float(d.get(k, 0.0)) for k in 'b s e t'.split()]
        self.state_pub.publish(msg)
        self._publish_ee_pose(d, msg.header.stamp)

    def _publish_ee_pose(self, d, stamp):
        # End-effector pose from the T:1051 x/y/z fields (mm) and t (rad).
        #
        # Yaw-convention ASSUMPTION (verify on the robot): the repo docs
        # (docs/arm_teach_repeat_plan.md, T:1041 home comment) pin only
        # "x,y,z mm + t rad" — they do not document the axis t rotates
        # about. We assume the RoArm T:1041/T:1051 frame is x forward,
        # y left, z up, and t is yaw about that z axis. The orientation
        # is therefore the euler quaternion (roll=0, pitch=0, yaw=t),
        # with yaw wrapped to [-pi, pi] and the quaternion components
        # clamped to [-1, 1] so TF consumers never see an out-of-range w.
        pose = PoseStamped()
        pose.header.stamp = stamp
        # RoArm fixed base frame (x fwd, y left, z up per T:1041 XYZ).
        pose.header.frame_id = 'arm_base_link'
        pose.pose.position.x = float(d.get('x', 0.0))
        pose.pose.position.y = float(d.get('y', 0.0))
        pose.pose.position.z = float(d.get('z', 0.0))
        yaw = math.atan2(math.sin(float(d.get('t', 0.0))),
                         math.cos(float(d.get('t', 0.0))))
        w = min(1.0, max(-1.0, math.cos(yaw / 2.0)))
        z = min(1.0, max(-1.0, math.sin(yaw / 2.0)))
        pose.pose.orientation.w = w
        pose.pose.orientation.z = z
        self.ee_pub.publish(pose)

    # --- ROS callbacks ---

    def _poll_timer(self):
        if not self.ready:
            return
        self._write('{"T":105}')

    def _goal_cb(self, msg):
        for name, rad in zip(msg.name, msg.position):
            if name in JOINT_IDS:
                spd = self.get_parameter('spd').value
                acc = self.get_parameter('acc').value
                self._write(
                    f'{{"T":101,"joint":{JOINT_IDS[name]},'
                    f'"rad":{rad},"spd":{spd},"acc":{acc}}}')

    def _ee_goal_cb(self, msg):
        # arm/ee_goals (geometry_msgs/PoseStamped): T:1041 XYZ direct
        # move in END-EFFECTOR space. position x/y/z are mm; t (rad) is
        # the yaw recovered from the message quaternion with the SAME
        # euler convention _publish_ee_pose uses (roll=0, pitch=0,
        # yaw=t about the arm z axis, w=cos(t/2), z=sin(t/2)):
        # t = 2*atan2(qz, qw) wrapped to [-pi, pi]. A goal carrying the
        # bridge's own arm/ee_pose quaternion therefore commands the
        # exact pose it was read from (round-trip consistent).
        # Non-yaw quaternion components are ignored.
        #
        # spd/acc come from the 'ee_spd'/'ee_acc' node params (defaults
        # 100/40; same scale as the T:1041 home move), NOT from the
        # message. T:1041 spd/acc units are undocumented in the repo;
        # assumed to be the same scale as T:101 spd/acc.
        #
        # T:1041 is a motion command, so it re-locks torque (see
        # _torque_cb): a goal published after a torque release
        # re-enables the motors and moves the arm.
        if not self.ready:
            now = time.time()
            if now - self._last_not_ready_warn >= 1.0:
                self.get_logger().warn(
                    'ignoring arm/ee_goals — arm not ready yet '
                    '(boot drain still running)')
                self._last_not_ready_warn = now
            return
        qz = min(1.0, max(-1.0, float(msg.pose.orientation.z)))
        qw = min(1.0, max(-1.0, float(msg.pose.orientation.w)))
        yaw = 2.0 * math.atan2(qz, qw)
        yaw = math.atan2(math.sin(yaw), math.cos(yaw))  # wrap to [-pi, pi]
        spd = self.get_parameter('ee_spd').value
        acc = self.get_parameter('ee_acc').value
        # Coalesce: only the latest goal survives; _ee_goal_flush
        # writes it on the 100 ms timer cadence.
        self._pending_ee_goal = {
            'T': 1041, 'x': float(msg.pose.position.x),
            'y': float(msg.pose.position.y),
            'z': float(msg.pose.position.z),
            't': yaw, 'spd': spd, 'acc': acc}

    def _ee_goal_flush(self):
        # Timer callback (100 ms): write the latest pending ee goal,
        # at most once per period. Runs only when ready; the write
        # lock keeps this off the RX thread's serial use.
        if not self.ready or self._pending_ee_goal is None:
            return
        goal = self._pending_ee_goal
        self._pending_ee_goal = None
        self._write(json.dumps(goal))

    def _torque_cb(self, req, resp):
        # T:210: cmd:0 releases the motors (hand-guiding / manual teach),
        # cmd:1 locks them. Any subsequent motion command re-locks, so
        # callers should re-enable torque before starting playback.
        if not self.ready:
            resp.success = False
            return resp
        cmd = 1 if req.enable else 0
        self._write(f'{{"T":210,"cmd":{cmd}}}')
        resp.success = True
        return resp


def main(args=None):
    rclpy.init(args=args)
    node = ArmBridge()
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
