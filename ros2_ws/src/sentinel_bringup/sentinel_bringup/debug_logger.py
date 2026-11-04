# Sets all loggers of the key nav2 nodes to 'debug', then exits. Runs
# as part of debug_nav.launch.py; give it ~90 s to find every service
# (nodes launched before this one may still be starting).
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import LoggerLevel
from rcl_interfaces.srv import SetLoggerLevels

TARGETS = [
    'controller_server',
    'planner_server',
    'bt_navigator',
    'behavior_server',
    'velocity_smoother',
    'smoother_server',
    'waypoint_follower',
    'amcl',
]
DEADLINE_S = 90.0


class DebugLogger(Node):

    def __init__(self):
        super().__init__('debug_logger')
        self.done = {name: False for name in TARGETS}
        self.clients = {}
        for name in TARGETS:
            self.clients[name] = self.create_client(
                SetLoggerLevels, '/%s/set_logger_levels' % name)
        self.deadline = self.get_clock().now()
        self.timer = self.create_timer(1.0, self.tick)

    def tick(self):
        for name, client in self.clients.items():
            if self.done[name]:
                continue
            if not client.wait_for_service(timeout_sec=0.5):
                continue
            req = SetLoggerLevels.Request()
            # Empty name sets the node's default level for all loggers.
            level = LoggerLevel(name='', level='debug')
            req.logger_levels = [level]
            future = client.call_async(req)
            future.add_done_callback(
                lambda f, n=name: self.on_result(n, f))

        now = self.get_clock().now()
        if (now - self.deadline).nanoseconds / 1e9 > DEADLINE_S:
            missing = [n for n, d in self.done.items() if not d]
            if missing:
                self.get_logger().warn(
                    'never reached: %s' % ', '.join(missing))
            self.get_logger().info('debug levels set, exiting')
            raise SystemExit

    def on_result(self, name, future):
        try:
            resp = future.result()
            ok = all(r.level == 'debug' for r in resp.logger_set)
            self.done[name] = True
            self.get_logger().info(
                '%s -> debug %s' % (name, 'OK' if ok else 'partial'))
        except Exception as e:  # noqa: BLE001 - service call failure
            self.get_logger().warn('%s failed: %s' % (name, e))


def main():
    rclpy.init()
    node = DebugLogger()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
