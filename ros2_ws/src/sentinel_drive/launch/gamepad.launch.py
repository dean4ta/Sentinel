# Gamepad drive stack: joy_node -> gamepad_twist -> cmd_vel_serial.
#
# Optional rosbag recording for tuning runs with the wheels on the
# ground:
#   ros2 launch sentinel_drive gamepad.launch.py record:=true
# Records /wheel_speeds and /encoder to data/gamepad_<date>; symlink
# data/gamepad_latest points at the newest recording. Analyze with:
#   python analyze_step_test.py ../data/gamepad_latest --wheel ALL -o ../data
# (no /step_events in this bag, so the per-step averages are skipped
# automatically).
#
# While record:=true, pressing X (cross) on the PS4 controller exits
# gamepad_twist, which shuts the whole launch down and closes the bag.
from datetime import datetime
import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, OpaqueFunction, Shutdown
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def arm_x_exit(context, gamepad_node):
    # Event handlers take no condition argument, so register the handler
    # only in recording mode. X-press exit must not arm otherwise: a dead
    # gamepad_twist would leave cmd_vel_serial driving the last twist.
    if str(context.launch_configurations['record']).lower() == 'true':
        context.register_event_handler(
            OnProcessExit(target_action=gamepad_node,
                          on_exit=[Shutdown()]))
    return []


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('sentinel_drive'),
        'config',
        'drive_params.yaml',
    )

    record = LaunchConfiguration('record', default='false')
    bag_out = LaunchConfiguration(
        'bag_out',
        default='data/gamepad_' + datetime.now().strftime('%Y-%m-%d_%H%M'))

    gamepad_node = Node(
        package='sentinel_drive',
        executable='gamepad_twist',
        name='gamepad_twist',
        parameters=[config, {'exit_on_x': record}],
    )

    bag = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '/wheel_speeds', '/encoder',
             '/wheel_consistency', '-o', bag_out],
        output='screen',
        condition=IfCondition(record),
    )

    # Symlink lives inside data/, so the target must not carry the
    # data/ prefix: ln resolves relative paths against the link's dir.
    # mkdir -p first: ln may run before rosbag creates data/.
    link_latest = ExecuteProcess(
        cmd=['bash', '-c',
             'mkdir -p data && ln -sfn "${1#data/}" data/gamepad_latest',
             '_', bag_out],
        output='screen',
        condition=IfCondition(record),
    )

    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{'autorepeat_rate': 20.0}],
        ),
        gamepad_node,
        Node(
            package='sentinel_drive',
            executable='cmd_vel_serial',
            name='cmd_vel_serial',
            parameters=[config],
        ),
        Node(
            package='sentinel_drive',
            executable='wheel_slip_detector',
            name='wheel_slip_detector',
            parameters=[config],
        ),
        bag,
        link_latest,
        OpaqueFunction(function=arm_x_exit, args=[gamepad_node]),
    ])
