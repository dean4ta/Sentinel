# Interactive /programmers_notes publisher. Keystrokes (terminal must
# have focus):
#   n        -> start a note; timestamp captured now, printed, and the
#               logger records it with the submitted text
#   <type>   -> note text (backspace works)
#   enter    -> publish note, back to idle
#   ctrl+x   -> cancel the draft, back to idle
#   ctrl+c   -> quit
#
# Run on the robot alongside the take-notes bringup:
#   ros2 run sentinel_bringup note_taker
import os
import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class NoteTaker(Node):

    def __init__(self):
        super().__init__('note_taker')
        self.pub = self.create_publisher(String, '/programmers_notes', 10)
        self.get_logger().info(
            'n=note, enter=submit, ctrl+x=cancel, ctrl+c=quit')

    def log_note(self, stamp_sec, text):
        # One log line carries both the n-press time and the text so the
        # console output alone lines up with a bag's /clock.
        self.get_logger().info('[t=%.3f] %s' % (stamp_sec, text))

    def publish_note(self, text):
        msg = String()
        msg.data = text
        self.pub.publish(msg)


def now_sec(node):
    t = node.get_clock().now()
    return t.nanoseconds / 1e9


def main():
    rclpy.init()
    node = NoteTaker()

    fd = sys.stdin.fileno()
    old_attrs = termios.tcgetattr(fd)
    tty.setcbreak(fd)

    drafting = False
    buf = ''
    stamp_sec = 0.0
    sys.stdout.write('> ')
    sys.stdout.flush()

    try:
        while rclpy.ok():
            readable, _, _ = select.select([sys.stdin], [], [], 0.05)
            if not readable:
                rclpy.spin_once(node, timeout_sec=0)
                continue

            ch = os.read(fd, 1)
            if not drafting:
                if ch == b'n':
                    drafting = True
                    buf = ''
                    stamp_sec = now_sec(node)
                    node.get_logger().info('note start t=%.3f' % stamp_sec)
                    sys.stdout.write('note> ')
                    sys.stdout.flush()
                elif ch == b'\x03':
                    raise KeyboardInterrupt
            else:
                if ch in (b'\n', b'\r'):
                    if buf.strip():
                        node.log_note(stamp_sec, buf)
                        node.publish_note(buf)
                    else:
                        node.get_logger().info('empty note dropped')
                    drafting = False
                    buf = ''
                    sys.stdout.write('> ')
                    sys.stdout.flush()
                elif ch == b'\x18':  # ctrl+x
                    node.get_logger().info('note cancelled')
                    drafting = False
                    buf = ''
                    sys.stdout.write('\n> ')
                    sys.stdout.flush()
                elif ch in (b'\x7f', b'\x08'):  # backspace
                    if buf:
                        buf = buf[:-1]
                        sys.stdout.write('\rnote> %s  ' % buf)
                        sys.stdout.write('\rnote> %s' % buf)
                        sys.stdout.flush()
                elif ch == b'\x03':
                    raise KeyboardInterrupt
                else:
                    text = ch.decode(errors='replace')
                    buf += text
                    sys.stdout.write(text)
                    sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_attrs)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
