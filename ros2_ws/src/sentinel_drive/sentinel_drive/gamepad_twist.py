# Gamepad /joy to /cmd_vel Twist publisher.
#
# Axes (PS4 / standard joy_node layout):
#   axes[1] = drive   (left stick Y, forward positive)
#   axes[0] = strafe  (left stick X, right positive)
#   axes[2] = rotate  (right stick X, right positive)
#
# Published Twist: linear.x = drive, linear.y = strafe,
# angular.z = rotate. Channel/sign compensation for Sentinel 1 wiring
# lives in cmd_vel_serial, not here.

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


class GamepadTwist(Node):

    def __init__(self):
        super().__init__('gamepad_twist')

        self.declare_parameter('deadzone', 0.05)
        self.declare_parameter('invert_strafe', False)
        self.declare_parameter('rotate_limit', 0.5)

        self.dz = self.get_parameter('deadzone').value
        self.invert_strafe = self.get_parameter('invert_strafe').value
        self.rotate_limit = self.get_parameter('rotate_limit').value

        self.create_subscription(Joy, '/joy', self.joy_cb, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info(
            f'Publishing /cmd_vel. deadzone={self.dz}, '
            f'invert_strafe={self.invert_strafe}, '
            f'rotate_limit={self.rotate_limit}'
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
        twist.linear.x = self._map(msg.axes[1])
        twist.linear.y = self._map(msg.axes[0])
        twist.angular.z = self._map(msg.axes[2])

        if self.invert_strafe:
            twist.linear.y = -twist.linear.y
        twist.angular.z = max(
            -self.rotate_limit, min(self.rotate_limit, twist.angular.z))

        self.pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(GamepadTwist())
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
