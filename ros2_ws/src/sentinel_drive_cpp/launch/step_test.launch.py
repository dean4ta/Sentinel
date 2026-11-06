# One-command step test with the C++ cmd_vel_serial. Same args (wheel,
# steps, sine), same bag topics, same config (sentinel_bringup's
# drive_params.yaml).
#
# Usage (from repo root, robot on blocks or floor -- WHEEL spins one wheel,
# ALL drives the robot):
#   ros2 launch sentinel_drive_cpp step_test.launch.py wheel:=FL \
#       steps:="0.2,5;0.5,10;1.0,5;0,1;-1.0,10" sine:="2.0,10,10"
#   ros2 launch sentinel_drive_cpp step_test.launch.py wheel:=ALL \
#       steps:="0.05,4;0.1,4" sine:=none
# sine = "amp_mps,period_s,duration_s" (none skips it); analyze with:
#   python analyze_step_test.py ../data/step_test_latest --wheel FL ...

from datetime import datetime

from launch import LaunchDescription
from launch.actions import ExecuteProcess, Shutdown, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

SHARE = FindPackageShare('sentinel_bringup')


def generate_launch_description():
    wheel = LaunchConfiguration('wheel', default='FL')
    steps = LaunchConfiguration(
        'steps', default='0.0,2;0.2,2;0.5,2;1.0,2;0,1;-1.0,2;-2.0,2;0,1')
    sine = LaunchConfiguration('sine', default='1.5,10,10')
    bag_out = LaunchConfiguration(
        'bag_out',
        default='data/step_test_' + datetime.now().strftime('%Y-%m-%d_%H%M'))

    serial_node = Node(
        package='sentinel_drive_cpp',
        executable='cmd_vel_serial',
        name='cmd_vel_serial',
        parameters=[[SHARE, '/config/drive_params.yaml']],
        output='screen',
    )

    bag = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '/wheel_speeds', '/encoder',
             '/step_events', '-o', bag_out],
        output='screen',
    )

    # Symlink lives inside data/, so the target must not carry the
    # data/ prefix: ln resolves relative paths against the link's dir.
    # mkdir -p first: ln may run before rosbag creates data/.
    link_latest = ExecuteProcess(
        cmd=['bash', '-c',
             'mkdir -p data && ln -sfn "${1#data/}" data/step_test_latest',
             '_', bag_out],
        output='screen',
    )

    test_node = TimerAction(
        period=2.0,  # let the bag start listening before step 1
        actions=[ExecuteProcess(
            cmd=['ros2', 'run', 'sentinel_bringup', 'speed_step_test',
                 wheel, steps, sine],
            output='screen',
            on_exit=[TimerAction(
                period=1.0,  # let the last step settle, then stop everything
                actions=[Shutdown()])],
        )],
    )

    return LaunchDescription([serial_node, bag, link_latest, test_node])
