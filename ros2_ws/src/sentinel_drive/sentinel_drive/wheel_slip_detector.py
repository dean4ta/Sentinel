# Wheel slip / contact-loss detection via leave-one-out mecanum
# consistency (see wheel_consistency.py for the math).
#
# /wheel_speeds (target + measured rpm, already forward-positive per
# cmd_vel_serial) -> WheelConsistency on /wheel_consistency: per-wheel
# confidence (1.0 trusted, 0.0 suspicious) + residual (m/s, sign-aware
# via suspicion's projection onto each wheel's direction of travel).
#
# Runs in the subscription callback: the filter advances exactly once
# per /wheel_speeds sample and goes quiet if cmd_vel_serial dies.
#
# Hardware sanity check: drive straight forward, all four measured
# values must come out positive.
#
# Asymmetric by design: a wheel faster than predicted is suspicious, a
# slower wheel stays trusted (spec assumption: wheels do not jam).

import rclpy
from rclpy.node import Node
from sentinel_msgs.msg import WheelSpeeds, WheelConsistency

from sentinel_drive.wheel_consistency import (
    WHEELS, ConfidenceFilter, leave_one_out, suspicion)


class WheelSlipDetector(Node):

    def __init__(self):
        super().__init__('wheel_slip_detector')

        # No geometry param needed: the check works directly in wL
        # (omega_z * wheelbase), which cancels out of the IK/FK
        # round-trip. lxy only matters to wheel_odometry.
        self.declare_parameter('wheel_circumference_m', 0.44)
        self.declare_parameter('residual_threshold_mps', 0.05)
        self.declare_parameter('residual_scale_mps', 0.15)
        self.declare_parameter('tracking_scale_mps', 0.30)
        self.declare_parameter('peer_margin_mps', 0.03)
        self.declare_parameter('alpha_suspicious', 0.20)
        self.declare_parameter('alpha_recover', 0.02)

        circ = self.get_parameter('wheel_circumference_m').value
        self.rpm_to_mps = circ / 60.0
        self.threshold = self.get_parameter('residual_threshold_mps').value
        self.scale = self.get_parameter('residual_scale_mps').value
        self.tracking_scale = self.get_parameter('tracking_scale_mps').value
        self.margin = self.get_parameter('peer_margin_mps').value

        self.filters = {w: ConfidenceFilter(
            alpha_suspicious=self.get_parameter('alpha_suspicious').value,
            alpha_recover=self.get_parameter('alpha_recover').value)
            for w in WHEELS}

        self.create_subscription(WheelSpeeds, '/wheel_speeds',
                                 self._cb, 10)
        self.pub = self.create_publisher(WheelConsistency,
                                         '/wheel_consistency', 10)

        self.get_logger().info(
            f'Slip detector: wheel_circumference={circ} m, '
            f'threshold={self.threshold} m/s, scale={self.scale}, '
            f'tracking_scale={self.tracking_scale}, '
            f'margin={self.margin}, alpha='
            f'{self.get_parameter("alpha_suspicious").value}/'
            f'{self.get_parameter("alpha_recover").value}')

    def _cb(self, msg):
        k = self.rpm_to_mps
        measured = {
            'FL': msg.fl_measured_rpm_filtered * k,
            'FR': msg.fr_measured_rpm_filtered * k,
            'BL': msg.bl_measured_rpm_filtered * k,
            'BR': msg.br_measured_rpm_filtered * k,
        }
        targets = {
            'FL': msg.fl_target_rpm * k,
            'FR': msg.fr_target_rpm * k,
            'BL': msg.bl_target_rpm * k,
            'BR': msg.br_target_rpm * k,
        }

        loo = leave_one_out(measured)
        errors = {w: abs(targets[w] - measured[w]) for w in WHEELS}

        out = WheelConsistency()
        out.header.stamp = self.get_clock().now().to_msg()
        for w in WHEELS:
            r = loo[w]['residual']
            peers = tuple(p for p in WHEELS if p != w)
            s = suspicion(r, measured[w], targets[w],
                          tuple(errors[p] for p in peers),
                          tuple(measured[p] for p in peers),
                          tuple(targets[p] for p in peers),
                          self.threshold, self.scale, self.tracking_scale,
                          self.margin)
            conf = self.filters[w].update(s)
            setattr(out, f'{w.lower()}_confidence', conf)
            setattr(out, f'{w.lower()}_residual', r)
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(WheelSlipDetector())
    except (SystemExit, KeyboardInterrupt):
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
