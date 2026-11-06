# Logs /programmers_notes to a timestamped file on the robot. Launched
# by sentinel_bringup.launch.py when take-notes:=true; note_taker.py
# publishes the notes. Each line: [ros_time] text, so the file lines up
# with a bag recorded against the same /clock.
import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ProgrammerNotation(Node):

    def __init__(self):
        super().__init__('programmer_notation')
        path = os.path.expanduser(
            '~/programmer_notes_%s.txt' % time.strftime('%Y%m%d_%H%M%S'))
        self.file = open(path, 'w')
        self.get_logger().info('logging notes to %s' % path)
        self.sub = self.create_subscription(
            String, '/programmers_notes', self.on_note, 10)

    def on_note(self, msg):
        now = self.get_clock().now()
        line = '[%.3f] %s\n' % (now.nanoseconds / 1e9, msg.data)
        self.file.write(line)
        self.file.flush()
        self.get_logger().info('note: %s' % msg.data)

    def destroy_node(self):
        self.file.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = ProgrammerNotation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
