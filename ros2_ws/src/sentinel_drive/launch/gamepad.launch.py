import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('sentinel_drive'),
        'config',
        'drive_params.yaml',
    )
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{'autorepeat_rate': 20.0}],
        ),
        Node(
            package='sentinel_drive',
            executable='gamepad_twist',
            name='gamepad_twist',
            parameters=[config],
        ),
        Node(
            package='sentinel_drive',
            executable='cmd_vel_serial',
            name='cmd_vel_serial',
            parameters=[config],
        ),
    ])
