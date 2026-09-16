from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{'autorepeat_rate': 20.0}],
        ),
        Node(
            package='sentinel_drive',
            executable='gamepad_serial',
            name='gamepad_serial',
        ),
    ])
