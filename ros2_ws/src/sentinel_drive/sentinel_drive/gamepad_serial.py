# Gamepad to Pico serial drive control.
#
# Subscribes /joy (PS4 via joy_node) and writes "drive,rotate,strafe\n"
# float triplets (-1..1) to the Pico over USB serial at a fixed rate.
# The Pico converts values to mecanum mixer PWM and enforces its own
# failsafe, so a continuous stream at >5Hz keeps the drive alive.
#
# Axes (PS4 / standard joy_node layout):
#   axes[1] = drive   (left stick Y, forward positive)
#   axes[0] = strafe  (left stick X, right positive)
#   axes[2] = rotate  (right stick X, right positive)
#
# Replaces the prior team's controller_to_pwm + pwm_serial_transmitter
# pair with a single node (no custom message, no intermediate topic).

import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


class GamepadSerial(Node):

    def __init__(self):
        super().__init__('gamepad_serial')

        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud', 460800)
        self.declare_parameter('deadzone', 0.05)
        self.declare_parameter('invert_strafe', True)
        self.declare_parameter('rotate_limit', 0.5)
        self.declare_parameter('publish_rate', 20.0)

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud').value
        self.dz = self.get_parameter('deadzone').value
        self.invert_strafe = self.get_parameter('invert_strafe').value
        self.rotate_limit = self.get_parameter('rotate_limit').value
        rate = self.get_parameter('publish_rate').value

        try:
            self.ser = serial.Serial(port, baud, timeout=0, write_timeout=0)
        except serial.SerialException as e:
            self.get_logger().error(f'Cannot open {port}: {e}')
            raise SystemExit(1)

        self.latest = (0.0, 0.0, 0.0)  # drive, rotate, strafe
        self.any_input = False

        self.create_subscription(Joy, '/joy', self.joy_cb, 10)
        self.create_timer(1.0 / rate, self.send_latest)

        self.get_logger().info(
            f'Sending to {port} @ {baud} at {rate} Hz. '
            f'deadzone={self.dz}, invert_strafe={self.invert_strafe}, '
            f'rotate_limit={self.rotate_limit}'
        )

    def _map(self, v):
        # Deadzone, then rescale so motion starts smoothly at the edge.
        if abs(v) <= self.dz:
            return 0.0
        v = (v - (1.0 if v > 0 else -1.0) * self.dz) / (1.0 - self.dz)
        return max(-1.0, min(1.0, v))

    def joy_cb(self, msg):
        if len(msg.axes) < 3:
            return
        drive = self._map(msg.axes[1])
        rotate = self._map(msg.axes[2])
        strafe = self._map(msg.axes[0])

        if self.invert_strafe:
            strafe = -strafe
        rotate = max(-self.rotate_limit, min(self.rotate_limit, rotate))

        self.latest = (drive, rotate, strafe)
        self.any_input = True

    def send_latest(self):
        drive, rotate, strafe = self.latest
        try:
            self.ser.write(f'{drive:.3f},{rotate:.3f},{strafe:.3f}\n'.encode())
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write failed: {e}', throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = GamepadSerial()
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
