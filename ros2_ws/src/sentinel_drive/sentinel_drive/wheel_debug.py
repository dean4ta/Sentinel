# Debug node: spin one wheel at a target RPM.
#
# Usage:
#   ros2 run sentinel_drive wheel_debug FL 30
#
# Publishes "FL 30" on /wheel_cmd at 20 Hz; cmd_vel_serial converts
# rpm -> normalized PWM (rpm / WHEEL_MAX_RPM) while the message is
# fresh. Single serial-port owner stays cmd_vel_serial.

import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

WHEELS = ('FL', 'FR', 'BL', 'BR')


class WheelDebug(Node):

    def __init__(self, wheel, rpm):
        super().__init__('wheel_debug')
        self.wheel = wheel.upper()
        self.rpm = float(rpm)
        if self.wheel not in WHEELS:
            self.get_logger().error(
                f'Unknown wheel {self.wheel}, use one of {WHEELS}')
            raise SystemExit(1)
        self.pub = self.create_publisher(String, '/wheel_cmd', 10)
        self.create_timer(0.05, self.tick)
        self.get_logger().info(
            f'Commanding {self.wheel} at {self.rpm} rpm on /wheel_cmd')

    def tick(self):
        msg = String()
        msg.data = f'{self.wheel} {self.rpm}'
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        if len(sys.argv) < 3:
            print('usage: ros2 run sentinel_drive wheel_debug WHEEL RPM')
            raise SystemExit(1)
        node = WheelDebug(sys.argv[1], sys.argv[2])
        rclpy.spin(node)
    except (SystemExit, KeyboardInterrupt):
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
