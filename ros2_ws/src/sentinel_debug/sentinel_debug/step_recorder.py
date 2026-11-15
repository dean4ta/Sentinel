# T012 step recorder: CSV capture of controller-level step tests.
#
# Records the topics the Mac-side analysis needs
# (utils/controller_level_analysis.py) as plain CSVs -- the Mac has no
# ROS and no rosbag2 python, so the bag alone is not enough. One CSV per
# topic in <out_dir>:
#   wheel_speeds.csv               /wheel_speeds (sentinel_msgs/WheelSpeeds)
#   wheel_odom.csv                 /wheel_odom (nav_msgs/Odometry)
#   wheel_imu_odom_filtered.csv    /wheel_imu_odom_filtered (nav_msgs/Odometry)
#   step_events.csv                /step_events (sentinel_msgs/StepEvent)
#
# CSV layout: header row = column names (first column `t`, seconds as
# float), one row per received message. WheelSpeeds and StepEvent carry
# no header stamp, so their `t` is the receive time; odometry `t` is the
# msg header stamp. Files are flushed every FLUSH_PERIOD_S and closed on
# shutdown, so a Ctrl-C'd run keeps its data.
#
# Launch (normally via sentinel_drive_cpp/step_test.launch.py, which
# passes level + out_dir):
#   ros2 run sentinel_debug step_recorder --ros-args -p level:=ffpid \
#       -p out_dir:=data/controller_level_ffpid
#
# Pure logic above the `import rclpy` seam so the Mac selftest
# (test/test_step_recorder.py) can AST-extract and exercise it with no
# ROS installed.

import math

FLUSH_PERIOD_S = 1.0

# sentinel_msgs/WheelSpeeds fields, in msg definition order.
WHEEL_SPEEDS_FIELDS = (
    'fl_target_rpm', 'fr_target_rpm', 'bl_target_rpm', 'br_target_rpm',
    'fl_measured_rpm_filtered', 'fr_measured_rpm_filtered',
    'bl_measured_rpm_filtered', 'br_measured_rpm_filtered',
    'fl_measured_rpm_raw', 'fr_measured_rpm_raw',
    'bl_measured_rpm_raw', 'br_measured_rpm_raw',
    'fl_cmd_rpm', 'fr_cmd_rpm', 'bl_cmd_rpm', 'br_cmd_rpm')

# Odometry columns (yaw computed from the quaternion, no tf2 dep).
ODOM_FIELDS = ('x', 'y', 'yaw', 'vx', 'wz')

# sentinel_msgs/StepEvent fields, in msg definition order.
STEP_EVENT_FIELDS = ('index', 'target_mps', 'sine')

WHEEL_SPEEDS_CSV = 'wheel_speeds.csv'
WHEEL_ODOM_CSV = 'wheel_odom.csv'
FILTERED_ODOM_CSV = 'wheel_imu_odom_filtered.csv'
STEP_EVENTS_CSV = 'step_events.csv'


def stamp_float(sec, nanosec):
    """msg header stamp (sec, nanosec) -> float seconds."""
    return float(sec) + float(nanosec) * 1e-9


def header_stamp_float(msg):
    """header.stamp of a stamped message -> float seconds."""
    return stamp_float(msg.header.stamp.sec, msg.header.stamp.nanosec)


def yaw_from_quat(q):
    """Yaw (rad) from a geometry_msgs-style quaternion (x, y, z, w
    attributes). Manual atan2 -- no tf2 dependency."""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wheel_speeds_row(msg, t):
    """CSV row for a WheelSpeeds msg: [t, all fields in msg order]."""
    return [t] + [float(getattr(msg, field)) for field in WHEEL_SPEEDS_FIELDS]


def odom_row(msg, t):
    """CSV row for a nav_msgs/Odometry msg: [t, x, y, yaw, vx, wz]."""
    position = msg.pose.pose.position
    return [t,
            float(position.x),
            float(position.y),
            yaw_from_quat(msg.pose.pose.orientation),
            float(msg.twist.twist.linear.x),
            float(msg.twist.twist.angular.z)]


def step_event_row(msg, t):
    """CSV row for a StepEvent msg: [t, index, target_mps, sine] (the
    bool as 0/1)."""
    return [t, int(msg.index), float(msg.target_mps), int(msg.sine)]


def default_out_dir(level):
    """CSV output dir for a controller level, relative to the CWD the
    node is launched from (repo root: data/)."""
    return 'data/controller_level_' + level


def make_header(fields):
    """CSV header line (no newline): `t` first, then the field names."""
    return 't,' + ','.join(fields)


def format_row(values):
    """CSV data line (no newline). Plain float str() = shortest
    round-trip repr; all recorder values are numeric, so no quoting."""
    return ','.join(str(value) for value in values)


class CsvSink:
    """Append-only CSV file. Header on open, flush() pushes the buffer,
    close() is final. Pure stdlib, Mac-testable."""

    def __init__(self, path, fields):
        self.path = path
        self._file = open(path, 'w', encoding='utf-8', newline='')
        self._file.write(make_header(fields) + '\n')

    def append(self, values):
        self._file.write(format_row(values) + '\n')

    def flush(self):
        self._file.flush()

    def close(self):
        self._file.close()


# ---- ROS wiring below (imports rclpy) ----

import os

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sentinel_msgs.msg import StepEvent, WheelSpeeds


class StepRecorder(Node):

    def __init__(self):
        super().__init__('step_recorder')
        # level first: the out_dir default is derived from it, so a
        # standalone `-p level:=off` run lands in the right dir.
        self.declare_parameter('level', 'ffpid')
        level = self.get_parameter('level').value
        self.declare_parameter('out_dir', default_out_dir(level))
        out_dir = self.get_parameter('out_dir').value
        os.makedirs(out_dir, exist_ok=True)

        self._sinks = {
            WHEEL_SPEEDS_CSV: CsvSink(
                os.path.join(out_dir, WHEEL_SPEEDS_CSV), WHEEL_SPEEDS_FIELDS),
            WHEEL_ODOM_CSV: CsvSink(
                os.path.join(out_dir, WHEEL_ODOM_CSV), ODOM_FIELDS),
            FILTERED_ODOM_CSV: CsvSink(
                os.path.join(out_dir, FILTERED_ODOM_CSV), ODOM_FIELDS),
            STEP_EVENTS_CSV: CsvSink(
                os.path.join(out_dir, STEP_EVENTS_CSV), STEP_EVENT_FIELDS),
        }
        self.create_subscription(
            WheelSpeeds, '/wheel_speeds', self._on_wheel_speeds, 10)
        self.create_subscription(
            Odometry, '/wheel_odom', self._on_wheel_odom, 10)
        self.create_subscription(
            Odometry, '/wheel_imu_odom_filtered', self._on_filtered_odom, 10)
        self.create_subscription(
            StepEvent, '/step_events', self._on_step_event, 10)
        self.create_timer(FLUSH_PERIOD_S, self._flush)

        self.get_logger().info(
            f'step_recorder: level={level}, writing CSVs to '
            f'{os.path.abspath(out_dir)}/')

    def _now_float(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_wheel_speeds(self, msg):
        # WheelSpeeds has no header stamp: receive time is the t column.
        self._sinks[WHEEL_SPEEDS_CSV].append(
            wheel_speeds_row(msg, self._now_float()))

    def _on_wheel_odom(self, msg):
        self._sinks[WHEEL_ODOM_CSV].append(
            odom_row(msg, header_stamp_float(msg)))

    def _on_filtered_odom(self, msg):
        self._sinks[FILTERED_ODOM_CSV].append(
            odom_row(msg, header_stamp_float(msg)))

    def _on_step_event(self, msg):
        # StepEvent has no header stamp: receive time is the t column.
        self._sinks[STEP_EVENTS_CSV].append(
            step_event_row(msg, self._now_float()))

    def _flush(self):
        for sink in self._sinks.values():
            sink.flush()

    def close(self):
        for sink in self._sinks.values():
            sink.close()


def main(args=None):
    rclpy.init(args=args)
    node = StepRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
