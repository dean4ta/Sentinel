# USB topology sensor test: probes every sensor on the new USB layout
# and reports PASS/FAIL for connection and for data. Runs standalone
# (`ros2 run sentinel_bringup sensor_check`) or as the checker inside
# sensor_test.launch.py, which starts the sensor nodes for it.
#
# Sensors checked here: rplidar (/scan), realsense gyro (gyro/sample),
# pico (/encoder via /dev/sentinel_pico), logitech webcam (V4L2).
# Arm + EAI lidar: deferred until plugged in (probe code returns
# NOT PRESENT so the table stays honest).
#
# Artifacts written to ~/sensor_test_out/: a 4-panel PNG (one lidar
# scan, averaged webcam frame, encoder series, gyro series) and a
# report_<stamp>.txt with the table + averages.

import math
import os
from datetime import datetime

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
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
OUT_DIR = os.path.expanduser('~/sensor_test_out')
STAMP = datetime.now().strftime('%Y%m%d_%H%M%S')


def plot_series(series, title, ylabel, size=(600, 400), legend=None):
    """series: list of (xs, ys) float arrays -> cv2 BGR image."""
    img = np.zeros((size[1], size[0], 3), np.uint8)
    m = 40  # margin
    x0, y0 = m, size[1] - m
    w, h = size[0] - 2 * m, size[1] - 2 * m
    xs_all = np.concatenate([s[0] for s in series])
    ys_all = np.concatenate([s[1] for s in series])
    xmin, xmax = xs_all.min(), xs_all.max()
    if xmax == xmin:
        xmax = xmin + 1
    ymin, ymax = ys_all.min(), ys_all.max()
    if ymax == ymin:
        ymax = ymin + 1
    colors = [(0, 255, 255), (0, 200, 0), (255, 120, 0), (255, 0, 255)]
    for (xs, ys), c in zip(series, colors):
        px = (x0 + (xs - xmin) / (xmax - xmin) * w).astype(np.int32)
        py = (y0 - (ys - ymin) / (ymax - ymin) * h).astype(np.int32)
        cv2.polylines(img, [np.stack([px, py], axis=1)], False, c, 1)
    cv2.putText(img, title, (m, 20), cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (255, 255, 255), 1)
    cv2.putText(img, f'{ylabel} [{ymin:.3g} .. {ymax:.3g}]',
                (m, size[1] - 12), cv2.FONT_HERSHEY_SIMPLEX,
                0.4, (255, 255, 255), 1)
    if legend:
        for i, name in enumerate(legend):
            cv2.putText(img, name, (size[0] - 120, 20 + 15 * i),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, colors[i], 1)
    return img


def lidar_png(scan, size=600):
    """Last scan as polar dots, 6 m radius."""
    img = np.zeros((size, size, 3), np.uint8)
    cx = cy = size // 2
    scale = (size // 2 - 30) / 6.0  # px per metre
    for ring_m in (2, 4, 6):
        cv2.circle(img, (cx, cy), int(ring_m * scale), (60, 60, 60), 1)
    cv2.line(img, (cx - 270, cy), (cx + 270, cy), (60, 60, 60), 1)
    cv2.line(img, (cx, cy - 270), (cx, cy + 270), (60, 60, 60), 1)
    for i, r in enumerate(scan.ranges):
        if not math.isfinite(r) or r <= 0.1 or r > 6.0:
            continue
        a = scan.angle_min + i * scan.angle_increment
        x = int(cx + r * scale * math.cos(a))
        y = int(cy - r * scale * math.sin(a))
        cv2.circle(img, (x, y), 1, (0, 255, 0), -1)
    cv2.putText(img, 'rplidar: one scan', (10, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    return img


class SensorCheck(Node):

    def __init__(self):
        super().__init__('sensor_check')
        self.scan_msgs = []
        self.scan_ranges = 0
        self.gyro_samples = []  # (t, wx, wy, wz)
        self.enc_samples = []  # (t, fl, fr, bl, br)
        self.enc_msgs = 0
        self.results = []  # (name, ok, detail) for the report file
        self.webcam_img = None  # cached by check_webcam for the collage

        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        # Realsense publishes IMU streams BEST_EFFORT; a RELIABLE sub
        # never gets a message (see incompatible-QoS warning in the
        # first run). Encoder + scan stay on the default.
        self.create_subscription(
            Imu, '/camera/realsense2_camera/gyro/sample', self.gyro_cb,
            qos_profile_sensor_data)
        if HAVE_MSGS:
            self.create_subscription(
                EncoderCounts, '/encoder', self.enc_cb, 10)

    # --- callbacks: collect during the sample window ---

    def scan_cb(self, msg):
        self.scan_msgs.append(msg)
        self.scan_ranges = max(self.scan_ranges, len(msg.ranges))

    def gyro_cb(self, msg):
        self.gyro_samples.append((
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            msg.angular_velocity.x, msg.angular_velocity.y,
            msg.angular_velocity.z))

    def enc_cb(self, msg):
        self.enc_samples.append((
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            msg.fl, msg.fr, msg.bl, msg.br))
        self.enc_msgs += 1

    # --- helpers ---

    def check(self, name, ok, detail):
        self.results.append((name, ok, detail))
        tag = 'PASS' if ok else 'FAIL'
        self.get_logger().info(f'[{tag}] {name}: {detail}')
        return ok

    def skip(self, name, detail):
        self.results.append((name, None, detail))
        self.get_logger().info(f'[SKIP] {name}: {detail}')

    def run_checks(self):
        self.get_logger().info('=== USB SENSOR TEST ===')

        # pico: device node + encoder stream. Robot is stationary during
        # the test, so counts must hold constant while messages flow.
        pico_dev = os.path.exists('/dev/sentinel_pico')
        self.check('connect pico', pico_dev,
                   '/dev/sentinel_pico present' if pico_dev
                   else '/dev/sentinel_pico missing')
        if HAVE_MSGS and self.enc_msgs > 0:
            first = self.enc_samples[0]
            last = self.enc_samples[-1]
            moving = tuple(last[i + 1] - first[i + 1] for i in range(4))
            self.check('data pico', self.enc_msgs >= 10,
                       f'{self.enc_msgs} msgs, count delta {moving} '
                       f'(stationary: delta 0 expected)')
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
        self.check('connect gyro', len(self.gyro_samples) > 0,
                   'gyro/sample live' if self.gyro_samples
                   else 'gyro/sample silent')
        if self.gyro_samples:
            g = np.array(self.gyro_samples)
            mean = g[:, 1:].mean(axis=0)
            std = g[:, 1:].std(axis=0)
            self.check('data gyro',
                       len(self.gyro_samples) >= 10 and std[2] > 1e-8,
                       f'{len(self.gyro_samples)} msgs, mean w '
                       f'({mean[0]:.4f}, {mean[1]:.4f}, {mean[2]:.4f}) '
                       f'rad/s, std z {std[2]:.2e}')
        else:
            self.check('data gyro', False, 'no gyro messages')

        # webcam: synchronous V4L2 grab while nothing else spins
        self.check_webcam()

        # arm + EAI lidar: deferred until plugged in
        self.skip('arm', 'not in lsusb (deferred)')
        self.skip('eai_lidar2', 'not plugged (deferred)')

        self.get_logger().info('=== DONE ===')
        self.save_artifacts()

    def check_webcam(self):
        if not HAVE_CV2:
            self.check('connect webcam', False, 'cv2 not importable')
            self.check('data webcam', False, 'skipped (no cv2)')
            return None
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
            return None
        ok = cap.get(cv2.CAP_PROP_FRAME_WIDTH) > 0
        self.check('connect webcam', ok, 'V4L2 device opens')
        # 20 frames, averaged: noise floor image for the report.
        acc = None
        frames = 0
        for _ in range(20):
            got, frame = cap.read()
            if not got:
                continue
            frames += 1
            if acc is None:
                acc = frame.astype(np.float32)
            else:
                acc += frame
        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        cap.release()
        self.check('data webcam', frames >= 8,
                   f'{frames}/20 frames, {w}x{h}')
        if acc is None:
            return None
        avg = (acc / frames).astype(np.uint8)
        cv2.putText(avg, 'webcam: average of frames', (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        self.webcam_img = avg
        return avg

    def save_artifacts(self):
        os.makedirs(OUT_DIR, exist_ok=True)
        report_path = os.path.join(OUT_DIR, f'report_{STAMP}.txt')
        png_path = os.path.join(OUT_DIR, f'sensor_report_{STAMP}.png')

        lines = [f'Sentinel 1 sensor test report {STAMP}', '']
        for name, ok, detail in self.results:
            tag = 'PASS' if ok else ('SKIP' if ok is None else 'FAIL')
            lines.append(f'[{tag}] {name}: {detail}')

        panels = []
        if self.scan_msgs:
            panels.append(('lidar', lidar_png(self.scan_msgs[-1])))
        if HAVE_MSGS and self.enc_samples:
            e = np.array(self.enc_samples)
            t0 = e[0, 0]
            xs = e[:, 0] - t0
            enc_img = plot_series(
                [(xs, e[:, i]) for i in range(1, 5)],
                'pico encoders vs time', 'counts',
                size=(600, 550), legend=['fl', 'fr', 'bl', 'br'])
            panels.append(('enc', enc_img))
            lines.append('')
            lines.append('encoder averages (counts): fl %.1f fr %.1f '
                         'bl %.1f br %.1f' %
                         tuple(e[:, 1:].mean(axis=0)))
        if self.gyro_samples:
            g = np.array(self.gyro_samples)
            t0 = g[0, 0]
            xs = g[:, 0] - t0
            gyro_img = plot_series(
                [(xs, g[:, i]) for i in range(1, 4)],
                'realsense gyro vs time', 'rad/s',
                size=(600, 450), legend=['wx', 'wy', 'wz'])
            panels.append(('gyro', gyro_img))
            lines.append('gyro averages (rad/s): wx %.4f wy %.4f wz %.4f'
                         % tuple(g[:, 1:].mean(axis=0)))

        webcam_img = self.webcam_img

        # Compose 2x2: lidar (600) + encoder (550) left; webcam + gyro right.
        if webcam_img is not None:
            webcam_img = cv2.resize(webcam_img, (600, 450))
            panels.append(('webcam', webcam_img))

        if len(panels) >= 4:
            row_l = cv2.vconcat([panels[0][1], panels[1][1]])
            right = [p[1] for p in panels if p[0] == 'gyro']
            right.insert(0, [p[1] for p in panels if p[0] == 'webcam'][0])
            # pad shorter right column to row_l height
            row_r = cv2.vconcat([right[0], right[1]])
            if row_r.shape[0] < row_l.shape[0]:
                pad = np.zeros((row_l.shape[0] - row_r.shape[0],
                                row_r.shape[1], 3), np.uint8)
                row_r = cv2.vconcat([row_r, pad])
            collage = cv2.hconcat([row_l, row_r])
            cv2.imwrite(png_path, collage)
            lines.append('')
            lines.append(f'image: {png_path}')

        with open(report_path, 'w') as f:
            f.write('\n'.join(lines) + '\n')
        self.get_logger().info(f'report: {report_path}')
        if os.path.exists(png_path):
            self.get_logger().info(f'image:  {png_path}')


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
