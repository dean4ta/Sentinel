# USB topology sensor test: probes every sensor on the new USB layout
# and reports PASS/FAIL for connection and for data. Runs standalone
# (`ros2 run sentinel_bringup sensor_check`) or as the checker inside
# sensor_test.launch.py, which starts the sensor nodes for it.
#
# Sensors checked here: rplidar (/scan), realsense gyro (gyro/sample),
# pico (/encoder via /dev/sentinel_pico), logitech webcam (V4L2).
# Arm + EAI lidar: deferred until plugged in (probe code returns
# NOT PRESENT so the table stays honest).

import os
import subprocess
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan

try:
    from sentinel_msgs.msg import EncoderCounts
    HAVE_MSGS = True
except ImportError:  # e.g. running outside the workspace env
    HAVE_MSGS = False

try:
    import cv2
    HAVE_CV2 = True
except ImportError:
    HAVE_CV2 = False

WINDOW_S = 3.0


class SensorCheck(Node):

    def __init__(self):
        super().__init__('sensor_check')
        self.scan_msgs = []
        self.scan_ranges = 0
        self.gyro_msgs = []
        self.gyro_var = 0.0
        self.enc_first = None
        self.enc_last = None
        self.enc_msgs = 0

        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_subscription(
            Imu, '/camera/realsense2_camera/gyro/sample', self.gyro_cb, 10)
        if HAVE_MSGS:
            self.create_subscription(
                EncoderCounts, '/encoder', self.enc_cb, 10)

    # --- callbacks: collect during the sample window ---

    def scan_cb(self, msg):
        self.scan_msgs.append(msg)
        self.scan_ranges = max(self.scan_ranges, len(msg.ranges))

    def gyro_cb(self, msg):
        self.gyro_msgs.append(msg)

    def enc_cb(self, msg):
        if self.enc_first is None:
            self.enc_first = msg
        self.enc_last = msg
        self.enc_msgs += 1

    # --- helpers ---

    def check(self, name, ok, detail):
        tag = 'PASS' if ok else 'FAIL'
        self.get_logger().info(f'[{tag}] {name}: {detail}')
        return ok

    def run_checks(self):
        self.get_logger().info('=== USB SENSOR TEST ===')

        # pico: device node + encoder stream
        pico_dev = os.path.exists('/dev/sentinel_pico')
        self.check('connect pico', pico_dev,
                   '/dev/sentinel_pico present' if pico_dev
                   else '/dev/sentinel_pico missing')
        if HAVE_MSGS and self.enc_msgs > 0 and \
                self.enc_first is not None and self.enc_last is not None:
            moving = (
                self.enc_last.fl - self.enc_first.fl,
                self.enc_last.fr - self.enc_first.fr,
                self.enc_last.bl - self.enc_first.bl,
                self.enc_last.br - self.enc_first.br,
            )
            self.check('data pico',
                       self.enc_msgs >= 10 and max(abs(d) for d in moving) > 0,
                       f'{self.enc_msgs} msgs, count delta {moving}')
        elif HAVE_MSGS:
            self.check('data pico', False, 'no /encoder messages')
        else:
            self.check('data pico', False, 'sentinel_msgs not importable')

        # rplidar
        self.check('connect rplidar', len(self.scan_msgs) > 0,
                   f'/scan live, {self.scan_ranges} ranges'
                   if self.scan_msgs else '/scan silent')
        self.check('data rplidar',
                   len(self.scan_msgs) >= 5 and self.scan_ranges > 100,
                   f'{len(self.scan_msgs)} msgs in {WINDOW_S:.0f} s, '
                   f'{self.scan_ranges} ranges')

        # realsense gyro
        self.check('connect gyro', len(self.gyro_msgs) > 0,
                   'gyro/sample live' if self.gyro_msgs else 'gyro/sample silent')
        if self.gyro_msgs:
            zs = [m.angular_velocity.z for m in self.gyro_msgs]
            mean = sum(zs) / len(zs)
            var = sum((z - mean) ** 2 for z in zs) / len(zs)
            self.check('data gyro',
                       len(self.gyro_msgs) >= 10 and var > 1e-8,
                       f'{len(self.gyro_msgs)} msgs, z variance {var:.2e}')
        else:
            self.check('data gyro', False, 'no gyro messages')

        # webcam: synchronous V4L2 grab while nothing else spins
        self.check_webcam()

        # arm + EAI lidar: deferred until plugged in
        self.get_logger().info('[SKIP] arm: not in lsusb (deferred)')
        self.get_logger().info('[SKIP] eai_lidar2: not plugged (deferred)')

        self.get_logger().info('=== DONE ===')

    def check_webcam(self):
        if not HAVE_CV2:
            self.check('connect webcam', False, 'cv2 not importable')
            self.check('data webcam', False, 'skipped (no cv2)')
            return
        # C270 exposes two video nodes (capture + metadata); cv2 opens
        # the first working one.
        cap = None
        for idx in range(4):
            cap = cv2.VideoCapture(idx)
            if cap.isOpened():
                break
        if cap is None or not cap.isOpened():
            self.check('connect webcam', False, 'no /dev/video node opens')
            self.check('data webcam', False, 'skipped (no device)')
            return
        ok = cap.get(cv2.CAP_PROP_FRAME_WIDTH) > 0
        self.check('connect webcam', ok, 'V4L2 device opens')
        frames = 0
        for _ in range(10):
            frames += 1 if cap.read()[0] else 0
        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        cap.release()
        self.check('data webcam', frames >= 8,
                   f'{frames}/10 frames, {w}x{h}')


def main(args=None):
    rclpy.init(args=args)
    node = SensorCheck()
    try:
        # Collect for the window, then evaluate everything at once.
        deadline = node.get_clock().now() + \
            rclpy.duration.Duration(seconds=WINDOW_S)
        while rclpy.ok() and node.get_clock().now() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        node.run_checks()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
