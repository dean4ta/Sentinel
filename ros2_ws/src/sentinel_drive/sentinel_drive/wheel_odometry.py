# Confidence-weighted mecanum wheel odometry.
#
# Subscribes /wheel_speeds (measured rpm per wheel, already
# forward-positive per cmd_vel_serial) and /wheel_consistency (per-wheel
# confidence from wheel_slip_detector). Each /wheel_speeds sample is
# turned into a body twist by ridge-regularised weighted least squares
# (see odometry_math.py), integrated into an odom-frame pose, and
# published as nav_msgs/Odometry on /wheel_odom.
#
# Degraded handling: when fewer than about three wheels are trusted, the
# unobservable twist directions fall back to the previous estimate and
# the twist covariance grows accordingly, so a downstream filter can
# down-weight this source. The slip detector assumes a single bad wheel,
# so low confidence on two wheels at once should be treated as "wheel
# odometry unreliable" rather than as a precise estimate.
#
# No TF is broadcast here. Let the eventual fusion node own odom->base.

import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from sentinel_msgs.msg import WheelConsistency, WheelSpeeds

from sentinel_drive.odometry_math import WHEELS, estimate_twist, integrate_pose

BIG_VAR = 1e6  # unobserved axes (z, roll, pitch)


class WheelOdometry(Node):

    def __init__(self):
        super().__init__('wheel_odometry')

        self.declare_parameter('wheel_circumference_m', 0.44)
        # half-track + half-wheelbase = 0.413/2 + 0.311/2 = 0.362 m.
        # NOTE: cmd_vel_serial's ik_wheelbase_m defaults to 0.35. They
        # should match, or commanded and measured yaw rates disagree.
        self.declare_parameter('lxy_m', 0.362)
        self.declare_parameter('weight_floor', 0.05)
        self.declare_parameter('ridge', 0.05)
        self.declare_parameter('wheel_noise_mps', 0.02)
        self.declare_parameter('degraded_min_eig', 0.5)
        self.declare_parameter('confidence_timeout_s', 0.2)
        self.declare_parameter('max_dt_s', 0.2)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        p = self.get_parameter
        self.k = p('wheel_circumference_m').value / 60.0
        self.lxy = p('lxy_m').value
        self.weight_floor = p('weight_floor').value
        self.ridge = p('ridge').value
        self.noise = p('wheel_noise_mps').value
        self.degraded_eig = p('degraded_min_eig').value
        self.conf_timeout = p('confidence_timeout_s').value
        self.max_dt = p('max_dt_s').value
        self.odom_frame = p('odom_frame').value
        self.base_frame = p('base_frame').value

        self.conf = None
        self.conf_rx = None
        self.prev_t = None
        self.prior = (0.0, 0.0, 0.0)
        self.pose = [0.0, 0.0, 0.0]          # x, y, yaw
        self.pvar = [0.0, 0.0]               # position var, yaw var
        self.was_degraded = False

        self.create_subscription(WheelConsistency, '/wheel_consistency',
                                 self._conf_cb, 10)
        self.create_subscription(WheelSpeeds, '/wheel_speeds',
                                 self._speeds_cb, 10)
        self.pub = self.create_publisher(Odometry, '/wheel_odom', 10)

        self.get_logger().info(
            f'Wheel odometry: circ={p("wheel_circumference_m").value} m, '
            f'lxy={self.lxy} m, floor={self.weight_floor}, '
            f'ridge={self.ridge}')

    def _conf_cb(self, msg):
        self.conf = {w: getattr(msg, f'{w.lower()}_confidence')
                     for w in WHEELS}
        self.conf_rx = self.get_clock().now()

    def _weights(self, now):
        """Confidences, or equal weights if the detector is silent."""
        if self.conf is None or self.conf_rx is None:
            return {w: 1.0 for w in WHEELS}
        age = (now - self.conf_rx).nanoseconds * 1e-9
        if age > self.conf_timeout:
            return {w: 1.0 for w in WHEELS}
        return self.conf

    def _stamp(self, msg):
        h = getattr(msg, 'header', None)
        if h is not None and (h.stamp.sec or h.stamp.nanosec):
            return Time.from_msg(h.stamp)
        return self.get_clock().now()

    def _speeds_cb(self, msg):
        now = self._stamp(msg)
        measured = {w: getattr(msg, f'{w.lower()}_measured_rpm_filtered')
                    * self.k for w in WHEELS}

        est = estimate_twist(
            measured, self._weights(self.get_clock().now()), self.prior,
            self.weight_floor, self.ridge, self.noise, self.degraded_eig)
        self.prior = (est.vx, est.vy, est.wL)
        omega = est.wL / self.lxy

        dt = 0.0
        if self.prev_t is not None:
            dt = (now - self.prev_t).nanoseconds * 1e-9
        self.prev_t = now
        if 0.0 < dt <= self.max_dt:
            x, y, yaw = integrate_pose(*self.pose, est.vx, est.vy,
                                       omega, dt)
            self.pose = [x, y, yaw]
            self.pvar[0] += (est.covariance[0, 0]
                             + est.covariance[1, 1]) * 0.5 * dt * dt
            self.pvar[1] += est.covariance[2, 2] / self.lxy ** 2 * dt * dt

        if est.degraded != self.was_degraded:
            self.was_degraded = est.degraded
            log = self.get_logger().warn if est.degraded \
                else self.get_logger().info
            log(f'Wheel odometry {"DEGRADED" if est.degraded else "healthy"}'
                f' (min_eig={est.min_eig:.2f})')

        self._publish(now, est, omega)

    def _publish(self, stamp, est, omega):
        out = Odometry()
        out.header.stamp = stamp.to_msg()
        out.header.frame_id = self.odom_frame
        out.child_frame_id = self.base_frame

        x, y, yaw = self.pose
        out.pose.pose.position.x = x
        out.pose.pose.position.y = y
        out.pose.pose.orientation.z = math.sin(0.5 * yaw)
        out.pose.pose.orientation.w = math.cos(0.5 * yaw)

        out.twist.twist.linear.x = est.vx
        out.twist.twist.linear.y = est.vy
        out.twist.twist.angular.z = omega

        pc = [0.0] * 36
        pc[0] = pc[7] = self.pvar[0]
        pc[35] = self.pvar[1]
        pc[14] = pc[21] = pc[28] = BIG_VAR
        out.pose.covariance = pc

        tc = [0.0] * 36
        tc[0] = float(est.covariance[0, 0])
        tc[7] = float(est.covariance[1, 1])
        tc[35] = float(est.covariance[2, 2]) / self.lxy ** 2
        tc[14] = tc[21] = tc[28] = BIG_VAR
        out.twist.covariance = tc

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(WheelOdometry())
    except (SystemExit, KeyboardInterrupt):
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
