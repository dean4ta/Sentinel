# C++ variant of sentinel_drive/launch/gamepad.launch.py: gamepad drive
# stack with the sentinel_drive_cpp nodes (cmd_vel_serial, slip detector,
# wheel odometry). gamepad_twist stays Python; config stays
# sentinel_drive's drive_params.yaml (single source of truth).
#
#   ros2 launch sentinel_drive_cpp gamepad.launch.py            # drive
#   ros2 launch sentinel_drive_cpp gamepad.launch.py record:=true
# Record mode: pressing X on the PS4 exits gamepad_twist, shutting the
# launch down and closing the bag cleanly.
from datetime import datetime
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, Shutdown
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PythonExpression
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
    # Declared explicitly: arm_x_exit reads launch_configurations['record']
    # directly, which only holds values for declared args.
    declare_record = DeclareLaunchArgument('record', default_value=record)
    bag_out = LaunchConfiguration(
        'bag_out',
        default='data/gamepad_' + datetime.now().strftime('%Y-%m-%d_%H%M'))
    # Space-separated topics for the recorder. Override for slim input
    # recordings meant for offline replay, e.g.:
    #   record:=true record_topics:="/wheel_speeds /encoder"
    record_topics = LaunchConfiguration(
        'record_topics',
        default='/wheel_speeds /encoder /wheel_consistency /wheel_odom')

    gamepad_node = Node(
        package='sentinel_drive',
        executable='gamepad_twist',
        name='gamepad_twist',
        parameters=[config, {'exit_on_x': record}],
    )

    bag = ExecuteProcess(
        cmd=['bash', '-c', PythonExpression(
            ["'ros2 bag record ' + '", record_topics, "' + ' -o ' + '",
             bag_out, "'"])],
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
        declare_record,
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{'autorepeat_rate': 20.0}],
        ),
        gamepad_node,
        Node(
            package='sentinel_drive_cpp',
            executable='cmd_vel_serial',
            name='cmd_vel_serial',
            parameters=[config],
        ),
        Node(
            package='sentinel_drive_cpp',
            executable='wheel_slip_detector',
            name='wheel_slip_detector',
            parameters=[config],
        ),
        Node(
            package='sentinel_drive_cpp',
            executable='wheel_odometry',
            name='wheel_odometry',
            parameters=[config],
        ),
        bag,
        link_latest,
        OpaqueFunction(function=arm_x_exit, args=[gamepad_node]),
    ])
