# One-command step test with the C++ cmd_vel_serial. Same args (wheel,
# steps, sine), same bag topics, same config (sentinel_bringup's
# drive_params.yaml) -- plus a controller_level arg (T012) that drives
# the controller comparison for final_submission/00_wheel_speed_controller:
#   off   = no controllers (open loop: cmd = target/WHEEL_MAX_RPM)
#   ff    = feed-forward only (cmd = kS*sign + kV*v_ref)
#   ffpid = feed-forward + PID + EMA filtered measurement (default)
#
# Also launches the odometry chain (wheel_slip_detector + wheel_odometry
# + robot_localization ekf_node) and sentinel_debug's step_recorder,
# which writes the per-level CSVs the Mac-side analysis reads
# (utils/controller_level_analysis.py): data/controller_level_<level>/.
#
# Usage (from repo root, robot on blocks or floor -- WHEEL spins one
# wheel, ALL drives the robot):
#   ros2 launch sentinel_drive_cpp step_test.launch.py wheel:=FL \
#       steps:="0.2,5;0.5,10;1.0,5;0,1;-1.0,10" sine:="2.0,10,10" \
#       controller_level:=ffpid
#   ros2 launch sentinel_drive_cpp step_test.launch.py wheel:=ALL \
#       steps:="0.05,4;0.1,4" sine:=none controller_level:=off
# sine = "amp_mps,period_s,duration_s" (none skips it); analyze with:
#   python analyze_step_test.py ../data/step_test_latest --wheel FL ...

from datetime import datetime

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, Shutdown,
    TimerAction)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

SHARE = FindPackageShare('sentinel_bringup')

# controller_level -> cmd_vel_serial parameter overrides, applied AFTER
# drive_params.yaml so they win over the file's pid_enabled /
# feed_forward_enabled defaults.
LEVEL_PARAMS = {
    'off': {'pid_enabled': False, 'feed_forward_enabled': False},
    'ff': {'pid_enabled': False, 'feed_forward_enabled': True},
    'ffpid': {'pid_enabled': True, 'feed_forward_enabled': True},
}


def level_nodes(context):
    """cmd_vel_serial with the resolved level's overrides, plus the CSV
    recorder writing to data/controller_level_<level>/."""
    level = LaunchConfiguration('controller_level').perform(context)
    serial = Node(
        package='sentinel_drive_cpp',
        executable='cmd_vel_serial',
        name='cmd_vel_serial',
        parameters=[[SHARE, '/config/drive_params.yaml'], LEVEL_PARAMS[level]],
        output='screen',
    )
    recorder = Node(
        package='sentinel_debug',
        executable='step_recorder',
        name='step_recorder',
        parameters=[{
            'level': level,
            'out_dir': 'data/controller_level_' + level,
        }],
        output='screen',
    )
    return [serial, recorder]


def generate_launch_description():
    controller_level = DeclareLaunchArgument(
        'controller_level',
        default_value='ffpid',
        choices=['off', 'ff', 'ffpid'],
        description='off: open loop; ff: feed-forward only; '
                    'ffpid: feed-forward + PID + EMA (default)')
    wheel = LaunchConfiguration('wheel', default='FL')
    steps = LaunchConfiguration(
        'steps', default='0.0,2;0.2,2;0.5,2;1.0,2;0,1;-1.0,2;-2.0,2;0,1')
    sine = LaunchConfiguration('sine', default='1.5,10,10')
    bag_out = LaunchConfiguration(
        'bag_out',
        default='data/step_test_' + datetime.now().strftime('%Y-%m-%d_%H%M'))

    # Odometry chain feeding the recorder's odom CSVs: slip confidence
    # (wheel_slip_detector) + confidence-weighted wheel odometry
    # (wheel_odometry) + the EKF fused with the D435i gyro. ekf.yaml
    # names the INPUT topics (odom0, imu0); the OUTPUT is the fixed
    # `odometry/filtered`, remapped to /wheel_imu_odom_filtered -- the
    # topic step_recorder subscribes and the same remap as
    # fusion.launch.py. Without the realsense driver the EKF still
    # publishes (odom-only); the analysis falls back to /wheel_odom
    # when the filtered CSV is absent.
    wheel_odometry = Node(
        package='sentinel_drive_cpp',
        executable='wheel_odometry',
        name='wheel_odometry',
        parameters=[[SHARE, '/config/drive_params.yaml']],
        output='screen',
    )
    slip_detector = Node(
        package='sentinel_drive_cpp',
        executable='wheel_slip_detector',
        name='wheel_slip_detector',
        parameters=[[SHARE, '/config/drive_params.yaml']],
        output='screen',
    )
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[[SHARE, '/config/ekf.yaml']],
        remappings=[('odometry/filtered', '/wheel_imu_odom_filtered')],
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

    return LaunchDescription([
        controller_level,
        wheel_odometry,
        slip_detector,
        ekf,
        OpaqueFunction(function=level_nodes),
        bag,
        link_latest,
        test_node,
    ])
