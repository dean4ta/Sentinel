# One-command step test: starts cmd_vel_serial, starts rosbag recording,
# then (2 s later, once the bag is listening) runs the step test node.
# The delay keeps every step's data in the bag from t=0.
#
# Usage (from repo root, robot on blocks):
#   ros2 launch sentinel_drive step_test.launch.py \
#       wheel:=FL steps:="0.2,5;0.5,10;1.0,5;0,1;-1.0,10"
#
# The launch ends by itself: 1 s after the test node finishes its last
# step, everything shuts down and the bag closes cleanly. A symlink
# data/step_test_latest always points at the newest recording:
#   python analyze_step_test.py ../data/step_test_latest --wheel FL ...

from datetime import datetime

from launch import LaunchDescription
from launch.actions import ExecuteProcess, Shutdown, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

SHARE = FindPackageShare('sentinel_drive')


def generate_launch_description():
    wheel = LaunchConfiguration('wheel', default='FL')
    steps = LaunchConfiguration(
        'steps', default='0.2,5;0.5,10;1.0,5;0,1;-1.0,10')
    bag_out = LaunchConfiguration(
        'bag_out',
        default='data/step_test_' + datetime.now().strftime('%Y-%m-%d_%H%M'))

    serial_node = Node(
        package='sentinel_drive',
        executable='cmd_vel_serial',
        name='cmd_vel_serial',
        parameters=[[SHARE, '/config/drive_params.yaml']],
        output='screen',
    )

    bag = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '/wheel_speeds', '/encoder',
             '-o', bag_out],
        output='screen',
    )

    # Symlink lives inside data/, so the target must not carry the
    # data/ prefix: ln resolves relative paths against the link's dir.
    link_latest = ExecuteProcess(
        cmd=['bash', '-c', 'ln -sfn "${1#data/}" data/step_test_latest',
             '_', bag_out],
        output='screen',
    )

    test_node = TimerAction(
        period=2.0,  # let the bag start listening before step 1
        actions=[ExecuteProcess(
            cmd=['ros2', 'run', 'sentinel_drive', 'speed_step_test',
                 wheel, steps],
            output='screen',
            on_exit=[TimerAction(
                period=1.0,  # let the last step settle, then stop everything
                actions=[Shutdown()])],
        )],
    )

    return LaunchDescription([serial_node, bag, link_latest, test_node])
