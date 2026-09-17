# /cmd_vel Twist to Pico serial drive control.
#
# Subscribes /cmd_vel and writes "drive,rotate,strafe\n" float triplets
# (-1..1) to the Pico over USB serial at a fixed rate. The Pico converts
# values to mecanum mixer PWM and enforces its own failsafe, so a
# continuous stream at >5Hz keeps the drive alive.
#
# Sentinel 1 wiring (observed 2026-09-17): 'x' signal drives robot strafe
# (Y = -x), 'r' drives forward/back (X = -r), 'y' drives rotation
# (R = -y). Remap + sign compensation lives here, at the hardware
# boundary, so /cmd_vel stays clean for future autonomy nodes.

import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


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
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.create_timer(1.0 / rate, self.send_latest)

        self.get_logger().info(f'Sending to {port} @ {baud} at {rate} Hz.')

    def cmd_vel_cb(self, msg):
        self.latest = msg

    def send_latest(self):
        # Wiring remap + sign compensation, see header comment.
        x_sig = -self.latest.linear.y
        r_sig = self.latest.linear.x
        y_sig = self.latest.angular.z
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
    except SystemExit:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
