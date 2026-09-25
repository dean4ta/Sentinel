# Step-response test node: drives one wheel through a list of
# (speed m/s, duration s) pairs by publishing /wheel_cmd.
#
# Usage:
#   ros2 run sentinel_drive speed_step_test FL "0.2,5;0.5,10;1.0,5;0,1;-1.0,10"
#
# Publish continuously (20 Hz) so cmd_vel_serial's /wheel_cmd freshness
# never expires mid-step. Record with rosbag while it runs:
#   ros2 bag record /wheel_speeds /encoder -o data/step_test_FL_<date>
#
# Robot on blocks. m/s -> wheel rpm uses the nominal 140 mm wheel
# circumference.

import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

WHEELS = ('FL', 'FR', 'BL', 'BR')
WHEEL_CIRCUMFERENCE_M = 0.44  # nominal 140 mm mecanum wheel


def parse_steps(text):
    steps = []
    for pair in text.split(';'):
        speed, dur = pair.split(',')
        steps.append((float(speed), float(dur)))
    return steps


class SpeedStepTest(Node):

    def __init__(self, wheel, steps):
        super().__init__('speed_step_test')
        self.wheel = wheel.upper()
        if self.wheel not in WHEELS:
            raise SystemExit(f'unknown wheel {wheel}, use one of {WHEELS}')
        self.steps = steps
        self.step_idx = 0
        self.step_start = self.get_clock().now()
        self.pub = self.create_publisher(String, '/wheel_cmd', 10)
        self.create_timer(0.05, self.tick)  # 20 Hz, within 0.5 s freshness
        self.get_logger().info(
            f'step test {self.wheel}: {steps} '
            f'(0.44 m circumference, robot on blocks)')

    def _send(self, rpm):
        msg = String()
        msg.data = f'{self.wheel} {rpm:.1f}'
        self.pub.publish(msg)

    def tick(self):
        now = self.get_clock().now()

        if self.step_idx >= len(self.steps):
            self._send(0.0)
            self.get_logger().info('all steps done')
            raise SystemExit(0)

        speed, dur = self.steps[self.step_idx]
        elapsed = (now - self.step_start).nanoseconds / 1e9
        if elapsed >= dur:
            self.step_idx += 1
            self.step_start = now
            if self.step_idx < len(self.steps):
                nxt, ndur = self.steps[self.step_idx]
                self.get_logger().info(
                    f'step {self.step_idx}: {nxt} m/s for {ndur} s')

        if self.step_idx < len(self.steps):
            speed, _ = self.steps[self.step_idx]
            self._send(speed / WHEEL_CIRCUMFERENCE_M * 60.0)


def main(args=None):
    rclpy.init(args=args)
    try:
        if len(sys.argv) < 3:
            print('usage: ros2 run sentinel_drive speed_step_test WHEEL '
                  '"speed,dur;speed,dur;..."')
            raise SystemExit(1)
        node = SpeedStepTest(sys.argv[1], parse_steps(sys.argv[2]))
        rclpy.spin(node)
    except (SystemExit, KeyboardInterrupt):
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
