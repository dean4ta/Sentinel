#!/usr/bin/env python3
# RoArm-M2-S serial <-> ROS bridge. One node: publishes arm/joint_states
# (from T:105 pose reads) and accepts arm/joint_goals (JointState ->
# per-joint T:101 move commands).
#
# Opening the serial port resets the ESP32 (DTR/RTS assert on open), so
# the boot/init chatter (~8 s, arm moves to init pose) is drained before
# any command goes out. Joints: base=1 shoulder=2 elbow=3 clamp(EOAT)=4.

import json
import threading
import time

import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

JOINT_IDS = {'base': 1, 'shoulder': 2, 'elbow': 3, 'clamp': 4}
JOINT_NAMES = ['base', 'shoulder', 'elbow', 'clamp']


class ArmBridge(Node):

    def __init__(self):
        super().__init__('arm_bridge')
        self.declare_parameter('port', '/dev/sentinel_arm')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('read_rate', 10.0)
        self.declare_parameter('spd', 50)   # T:101 steps/s
        self.declare_parameter('acc', 10)   # T:101 acc units

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        self.ser = serial.Serial(port, baud, timeout=0.5)
        self.ser.dtr = False
        self.ser.rts = False
        self.write_lock = threading.Lock()

        # RX thread: drains the boot sequence, then parses feedback.
        self.rx_thread = threading.Thread(target=self._rx_loop,
                                          daemon=True)
        self.rx_thread.start()

        self.state_pub = self.create_publisher(
            JointState, 'arm/joint_states', 10)
        self.goal_sub = self.create_subscription(
            JointState, 'arm/joint_goals', self._goal_cb, 10)
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

    # --- ROS callbacks ---

    def _poll_timer(self):
        self._write('{"T":105}')

    def _goal_cb(self, msg):
        for name, rad in zip(msg.name, msg.position):
            if name in JOINT_IDS:
                spd = self.get_parameter('spd').value
                acc = self.get_parameter('acc').value
                self._write(
                    f'{{"T":101,"joint":{JOINT_IDS[name]},'
                    f'"rad":{rad},"spd":{spd},"acc":{acc}}}')


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
