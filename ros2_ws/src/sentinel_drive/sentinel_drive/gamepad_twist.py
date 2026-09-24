# Gamepad /joy to /cmd_vel Twist publisher.
#
# Axes (PS4 / standard joy_node layout):
#   axes[1] = drive   (left stick Y, forward positive)
#   axes[0] = strafe  (left stick X, right positive)
#   axes[2] = rotate  (right stick X, right positive)
#
# Published Twist (SI units): linear.x = drive (m/s), linear.y = strafe
# (m/s), angular.z = rotate (rad/s), scaled by the V_MAX ceiling constants
# shared with cmd_vel_serial. Channel/sign compensation for Sentinel 1
# wiring lives in cmd_vel_serial, not here.

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

from sentinel_drive.cmd_vel_serial import (
    V_MAX_ANGULAR_RADPS, V_MAX_LINEAR_MPS)


class GamepadTwist(Node):

    def __init__(self):
        super().__init__('gamepad_twist')

        self.declare_parameter('deadzone', 0.05)
        self.declare_parameter('invert_strafe', False)
        self.declare_parameter('rotate_limit', 2.5)  # rad/s
        self.declare_parameter('speed_scale', 1.0)

        self.dz = self.get_parameter('deadzone').value
        self.invert_strafe = self.get_parameter('invert_strafe').value
        self.rotate_limit = self.get_parameter('rotate_limit').value
        self.speed_scale = min(
            1.0, max(0.0, self.get_parameter('speed_scale').value))

        self.create_subscription(Joy, '/joy', self.joy_cb, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info(
            f'Publishing /cmd_vel. deadzone={self.dz}, '
            f'invert_strafe={self.invert_strafe}, '
            f'rotate_limit={self.rotate_limit}, '
            f'speed_scale={self.speed_scale}'
        )

    def _map(self, v):
        # Deadzone, then rescale so motion starts smoothly at the edge.
        if abs(v) <= self.dz:
            return 0.0
        v = (v - (1.0 if v > 0 else -1.0) * self.dz) / (1.0 - self.dz)
        return max(-1.0, min(1.0, v))

    def joy_cb(self, msg):
        if len(msg.axes) < 3:
            return

        twist = Twist()
        twist.linear.x = self._map(msg.axes[1]) * V_MAX_LINEAR_MPS
        twist.linear.y = self._map(msg.axes[0]) * V_MAX_LINEAR_MPS
        twist.angular.z = self._map(msg.axes[2]) * V_MAX_ANGULAR_RADPS

        if self.invert_strafe:
            twist.linear.y = -twist.linear.y
        twist.angular.z = max(
            -self.rotate_limit, min(self.rotate_limit, twist.angular.z))

        twist.linear.x *= self.speed_scale
        twist.linear.y *= self.speed_scale
        twist.angular.z *= self.speed_scale

        self.pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(GamepadTwist())
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
