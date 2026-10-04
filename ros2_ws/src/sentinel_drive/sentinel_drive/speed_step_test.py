# Step-response test: drives wheels through (speed m/s, duration s)
# pairs, then an optional sine sweep "amp_mps,period_s,duration_s".
#
# WHEEL = FL/FR/BL/BR publishes /wheel_cmd (one wheel at a time).
# WHEEL = ALL publishes Twist on /cmd_vel (linear.x only), all four
# wheels together through the mecanum IK. Robot on blocks either way.
#
# Usage:
#   ros2 run sentinel_drive speed_step_test FL "0.2,5;0.5,10;1.0,5;0,1;-1.0,10" "2.0,10,10"
#   ros2 run sentinel_drive speed_step_test ALL "0.2,5;0.5,10" ""
#
# Publishes continuously (20 Hz) so /wheel_cmd freshness never expires.
# Record alongside: ros2 bag record /wheel_speeds /encoder -o data/step_test_<date>
# Single-wheel rpm uses the nominal 0.44 m circumference; ALL mode goes
# through V_MAX_LINEAR_MPS scaling (same to within 1%).

import math
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sentinel_msgs.msg import StepEvent

WHEELS = ('FL', 'FR', 'BL', 'BR')
WHEEL_CIRCUMFERENCE_M = 0.44  # nominal 140 mm mecanum wheel


def parse_steps(text):
    steps = []
    for pair in text.split(';'):
        speed, dur = pair.split(',')
        steps.append((float(speed), float(dur)))
    return steps


def parse_sine(text):
    amp, period, dur = text.split(',')
    return float(amp), float(period), float(dur)


class SpeedStepTest(Node):

    def __init__(self, wheel, steps, sine=None):
        super().__init__('speed_step_test')
        self.wheel = wheel.upper()
        if self.wheel not in WHEELS and self.wheel != 'ALL':
            raise SystemExit(
                f'unknown wheel {wheel}, use one of {WHEELS} or ALL')
        self.steps = steps
        self.sine = sine  # (amp m/s, period s, duration s) or None
        self.phase = 'steps'
        self.step_idx = 0
        self.step_start = self.get_clock().now()
        if self.wheel == 'ALL':
            self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        else:
            self.pub = self.create_publisher(String, '/wheel_cmd', 10)
        self.event_pub = self.create_publisher(StepEvent, '/step_events', 10)
        self._publish_event(0, self.steps[0][0])
        self.create_timer(0.05, self.tick)  # 20 Hz, within 0.5 s freshness
        self.get_logger().info(
            f'step test {self.wheel}: {steps} '
            f'(0.44 m circumference, robot on blocks)')

    def _send(self, mps):
        if self.wheel == 'ALL':
            # ALL: linear.x -> same surface speed on all four wheels.
            msg = Twist()
            msg.linear.x = mps
        else:
            msg = String()
            msg.data = f'{self.wheel} {mps / WHEEL_CIRCUMFERENCE_M * 60.0:.1f}'
        self.pub.publish(msg)

    def _publish_event(self, index, mps, sine=False):
        msg = StepEvent()
        msg.index = index
        msg.target_mps = mps
        msg.sine = sine
        self.event_pub.publish(msg)

    def tick(self):
        now = self.get_clock().now()

        if self.phase == 'sine':
            self._tick_sine(now)
            return

        if self.step_idx >= len(self.steps):
            if self.sine is not None:
                self.phase = 'sine'
                self.step_start = now
                amp, period, dur = self.sine
                self.get_logger().info(
                    f'sine: {amp} m/s amp, {period} s period, {dur} s')
                self._publish_event(-1, 0.0, sine=True)
                return
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
                self._publish_event(self.step_idx, nxt)

        if self.step_idx < len(self.steps):
            speed, _ = self.steps[self.step_idx]
            self._send(speed)

    def _tick_sine(self, now):
        amp, period, dur = self.sine
        t = (now - self.step_start).nanoseconds / 1e9
        if t >= dur:
            self._send(0.0)
            self.get_logger().info('sine done')
            raise SystemExit(0)
        v = amp * math.sin(2 * math.pi * t / period)
        self._send(v)


def main(args=None):
    rclpy.init(args=args)
    try:
        if len(sys.argv) < 3:
            print('usage: ros2 run sentinel_drive speed_step_test '
                  'WHEEL|ALL "speed,dur;speed,dur;..." ["amp,period,dur"]')
            raise SystemExit(1)
        sine = parse_sine(sys.argv[3]) if len(sys.argv) > 3 else None
        node = SpeedStepTest(sys.argv[1], parse_steps(sys.argv[2]), sine)
        rclpy.spin(node)
    except (SystemExit, KeyboardInterrupt):
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
