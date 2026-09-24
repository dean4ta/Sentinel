# /cmd_vel Twist <-> Pico serial gateway.
#
# Writes: subscribes /cmd_vel and writes "drive,rotate,strafe\n" float
# triplets (-1..1) to the Pico over USB serial at a fixed rate. The Pico
# converts values to mecanum mixer PWM and enforces its own failsafe, so
# a continuous stream at >5Hz keeps the drive alive.
#
# Reads: drains "FL <n>" / "FR <n>" / "BL <n>" / "BR <n>" encoder lines
# from the Pico and publishes them as sentinel_drive/EncoderCounts on
# /encoder. This node must be the only process holding the serial port,
# so it owns both directions.
#
# Sentinel 1 wiring (observed 2026-09-17): 'x' signal drives robot strafe
# (Y = -x), 'r' drives forward/back (X = -r), 'y' drives rotation
# (R = -y). Remap + sign compensation lives here, at the hardware
# boundary, so /cmd_vel stays clean for future autonomy nodes.

import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sentinel_msgs.msg import EncoderCounts

ENCODER_NAMES = ('FL', 'FR', 'BL', 'BR')

# /cmd_vel carries SI units (m/s, rad/s); these ceiling constants normalize
# to the [-1, 1] wire protocol. Both are first-pass estimates pending
# real-robot calibration, not trusted exact values.
V_MAX_LINEAR_MPS = 2.29   # TODO: measure. Theoretical no-load max at 12 V:
                          # 5203 19.2:1 motor (312 RPM output) x 140 mm
                          # mecanum wheel circumference. Loaded max lower.
V_MAX_ANGULAR_RADPS = 5.0  # TODO: measure. Placeholder pending a timed
                           # full-speed spin test (rad/s at commanded max
                           # rotate) — no clean formula; mixing happens in
                           # the hardware mecanum mixer board, not here.


def _clamp(v):
    return max(-1.0, min(1.0, v))


class CmdVelSerial(Node):

    def __init__(self):
        super().__init__('cmd_vel_serial')

        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud', 460800)
        self.declare_parameter('publish_rate', 20.0)

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud').value
        rate = self.get_parameter('publish_rate').value

        try:
            self.ser = serial.Serial(port, baud, timeout=0, write_timeout=0)
        except serial.SerialException as e:
            self.get_logger().error(f'Cannot open {port}: {e}')
            raise SystemExit(1)

        self.latest = Twist()
        self.rx_buf = b''
        self.counts = {name: 0 for name in ENCODER_NAMES}
        self.encoder_pub = self.create_publisher(EncoderCounts, '/encoder', 10)

        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.create_timer(1.0 / rate, self.tick)

        self.get_logger().info(
            f'Gateway on {port} @ {baud}: writing at {rate} Hz, '
            f'publishing EncoderCounts on /encoder.')

    def cmd_vel_cb(self, msg):
        self.latest = msg

    def tick(self):
        # One timer drives both directions: drain whatever the Pico sent
        # (>= its 10 Hz report rate), then write the latest /cmd_vel.
        self.drain_encoders()
        self.send_latest()

    def drain_encoders(self):
        try:
            chunk = self.ser.read(1024)
        except serial.SerialException as e:
            self.get_logger().error(
                f'Serial read failed: {e}', throttle_duration_sec=2.0)
            return
        if not chunk:
            return

        self.rx_buf += chunk
        lines = self.rx_buf.split(b'\n')
        self.rx_buf = lines[-1]  # keep the partial tail for next tick

        updated = False
        for line in lines[:-1]:
            parts = line.decode().split()
            if len(parts) != 2 or parts[0] not in self.counts:
                continue
            try:
                value = int(parts[1])
            except ValueError:
                continue
            self.counts[parts[0]] = value
            updated = True
        if updated:
            self.publish_counts()

    def publish_counts(self):
        msg = EncoderCounts()
        msg.fl = self.counts['FL']
        msg.fr = self.counts['FR']
        msg.bl = self.counts['BL']
        msg.br = self.counts['BR']
        self.encoder_pub.publish(msg)

    def send_latest(self):
        # Wiring remap + sign compensation, see header comment.
        # Normalize SI velocities against the ceiling constants before
        # writing the [-1, 1] wire protocol.
        x_sig = _clamp(-self.latest.linear.y / V_MAX_LINEAR_MPS)
        r_sig = _clamp(self.latest.linear.x / V_MAX_LINEAR_MPS)
        y_sig = _clamp(self.latest.angular.z / V_MAX_ANGULAR_RADPS)
        try:
            self.ser.write(
                f'{x_sig:.3f},{r_sig:.3f},{y_sig:.3f}\n'.encode())
        except serial.SerialException as e:
            self.get_logger().error(
                f'Serial write failed: {e}', throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = CmdVelSerial()
        rclpy.spin(node)
    except (SystemExit, KeyboardInterrupt):
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
