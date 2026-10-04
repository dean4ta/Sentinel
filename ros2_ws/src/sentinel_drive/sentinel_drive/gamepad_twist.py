# Gamepad /joy -> /cmd_vel Twist (SI: m/s, rad/s).
#
# Axes (PS4 / joy_node layout): 1 = drive (left Y), 0 = strafe (left X),
# 2 = rotate (right X). Speeds scale by the V_MAX_* ceilings shared with
# cmd_vel_serial; channel/sign compensation for the wiring lives there.

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
        self.declare_parameter('invert_rotate', False)
        self.declare_parameter('rotate_limit', 2.5)  # rad/s
        self.declare_parameter('speed_scale', 1.0)
        # Any button ends the run: the sticks do the driving and no
        # button has another job, so matching one PS4 button index is
        # not worth the mapping fragility.
        self.declare_parameter('exit_on_x', False)

        self.dz = self.get_parameter('deadzone').value
        self.invert_strafe = self.get_parameter('invert_strafe').value
        self.invert_rotate = self.get_parameter('invert_rotate').value
        self.rotate_limit = self.get_parameter('rotate_limit').value
        self.speed_scale = min(
            1.0, max(0.0, self.get_parameter('speed_scale').value))
        # launch passes record:= as a string ('true'/'false'); accept
        # either type.
        self.exit_on_x = (
            str(self.get_parameter('exit_on_x').value).lower() == 'true')
        self.exit_requested = False

        self.create_subscription(Joy, '/joy', self.joy_cb, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # Raise SystemExit from a timer, not the subscription callback:
        # timer exceptions reliably tear down rclpy.spin.
        self.create_timer(0.1, self._tick_exit)

        self.get_logger().info(
            f'Publishing /cmd_vel. deadzone={self.dz}, '
            f'invert_strafe={self.invert_strafe}, '
            f'invert_rotate={self.invert_rotate}, '
            f'rotate_limit={self.rotate_limit}, '
            f'speed_scale={self.speed_scale}, '
            f'exit_on_x={self.exit_on_x}'
        )

    def _tick_exit(self):
        if self.exit_requested:
            self.get_logger().info('button pressed: exiting')
            raise SystemExit(0)

    def _map(self, v):
        # Deadzone, then rescale so motion starts smoothly at the edge.
        if abs(v) <= self.dz:
            return 0.0
        v = (v - (1.0 if v > 0 else -1.0) * self.dz) / (1.0 - self.dz)
        return max(-1.0, min(1.0, v))

    def joy_cb(self, msg):
        if len(msg.axes) < 3:
            return

        if self.exit_on_x and any(msg.buttons):
            self.get_logger().info(
                f'buttons pressed: '
                f'{[i for i, b in enumerate(msg.buttons) if b]}')
            self.exit_requested = True
            return

        twist = Twist()
        twist.linear.x = self._map(msg.axes[1]) * V_MAX_LINEAR_MPS
        twist.linear.y = self._map(msg.axes[0]) * V_MAX_LINEAR_MPS
        twist.angular.z = self._map(msg.axes[2]) * V_MAX_ANGULAR_RADPS

        if self.invert_strafe:
            twist.linear.y = -twist.linear.y
        twist.angular.z = max(
            -self.rotate_limit, min(self.rotate_limit, twist.angular.z))
        if self.invert_rotate:
            twist.angular.z = -twist.angular.z

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
