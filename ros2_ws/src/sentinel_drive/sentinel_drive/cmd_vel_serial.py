# /cmd_vel Twist <-> Pico serial gateway.
#
# Writes: subscribes /cmd_vel and writes "fl,fr,bl,br\n" float wheel
# commands (-1..1) to the Pico over USB serial at a fixed rate. The Pico
# converts values to per-wheel PWM and enforces its own failsafe, so
# a continuous stream at >5Hz keeps the drive alive.
#
# Also subscribes /wheel_cmd (std_msgs/String, "FL 30" = 30 rpm) for
# per-wheel debug; it overrides /cmd_vel while fresh (< 0.5 s silence
# falls back to twist).
#
# Reads: drains "FL <n>" / "FR <n>" / "BL <n>" / "BR <n>" encoder lines
# from the Pico and publishes them as sentinel_drive/EncoderCounts on
# /encoder. This node must be the only process holding the serial port,
# so it owns both directions.
#
# Direct drive, mecanum mixer bypassed (2026-09-24): FR -> GP18,
# BR -> GP19, FL -> GP20, BL -> GP21.

import serial
import rclpy
import rclpy.duration
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
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
WHEEL_MAX_RPM = 312.0  # TODO: measure. 5203 19.2:1 no-load output RPM,
                       # used by /wheel_cmd debug commands (rpm -> [-1, 1]).

# Per-wheel PWM direction: +1 = positive command spins wheel forward.
# FR/BR flipped (observed 2026-09-24): right-side motor controllers mount
# with opposite polarity. Applied at the serial boundary so both /cmd_vel
# and /wheel_cmd get corrected.
WHEEL_DIRECTIONS = {'FL': 1.0, 'FR': -1.0, 'BL': 1.0, 'BR': -1.0}


def _clamp(v):
    return max(-1.0, min(1.0, v))


class CmdVelSerial(Node):

    def __init__(self):
        super().__init__('cmd_vel_serial')

        self.declare_parameter('serial_port', '/dev/sentinel_pico')
        self.declare_parameter('baud', 460800)
        self.declare_parameter('publish_rate', 50.0)

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

        # Per-wheel debug state: /wheel_cmd overrides /cmd_vel while fresh.
        self.mode = 'twist'  # 'twist' or 'wheels'
        self.wheel_targets = {name: 0.0 for name in ENCODER_NAMES}
        self.last_wheel_cmd = self.get_clock().now()

        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.create_subscription(String, '/wheel_cmd', self.wheel_cmd_cb, 10)
        self.create_timer(1.0 / rate, self.tick)

        self.get_logger().info(
            f'Gateway on {port} @ {baud}: writing at {rate} Hz, '
            f'publishing EncoderCounts on /encoder.')

    def cmd_vel_cb(self, msg):
        self.latest = msg

    def wheel_cmd_cb(self, msg):
        # "FL 30" debug format: one wheel at target RPM, rest zeroed.
        parts = msg.data.split()
        if len(parts) != 2 or parts[0].upper() not in self.wheel_targets:
            return
        try:
            rpm = float(parts[1])
        except ValueError:
            return
        for name in self.wheel_targets:
            self.wheel_targets[name] = 0.0
        self.wheel_targets[parts[0].upper()] = _clamp(rpm / WHEEL_MAX_RPM)
        self.last_wheel_cmd = self.get_clock().now()
        self.mode = 'wheels'

    def tick(self):
        # One timer drives both directions: drain whatever the Pico sent
        # (>= its 50 Hz report rate), then write the latest /cmd_vel.
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
        # Wheel debug commands win while fresh; fall back to /cmd_vel
        # after 0.5 s of silence so a dead debug node can't hold motion.
        if self.mode == 'wheels' and (
                self.get_clock().now() - self.last_wheel_cmd
                > rclpy.duration.Duration(seconds=0.5)):
            self.mode = 'twist'

        if self.mode == 'wheels':
            fl = self.wheel_targets['FL']
            fr = self.wheel_targets['FR']
            bl = self.wheel_targets['BL']
            br = self.wheel_targets['BR']
        else:
            # Forward drive only: same normalized command to all wheels.
            # TODO: full mecanum inverse kinematics (strafe, rotate).
            # TODO: per-wheel gains / speed controller here
            #       (target speed -> PWM -> encoder feedback).
            fwd = _clamp(self.latest.linear.x / V_MAX_LINEAR_MPS)
            fl = fr = bl = br = fwd
        fl *= WHEEL_DIRECTIONS['FL']
        fr *= WHEEL_DIRECTIONS['FR']
        bl *= WHEEL_DIRECTIONS['BL']
        br *= WHEEL_DIRECTIONS['BR']
        try:
            self.ser.write(f'{fl:.3f},{fr:.3f},{bl:.3f},{br:.3f}\n'.encode())
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
