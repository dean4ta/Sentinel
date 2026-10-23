# SLAM mapping stack: RPLIDAR A1 driver, base_link -> laser static TF
# (lidar mounted 4 in above base_link, its X facing -X of robot travel:
# yaw 180 deg), wheel-odom + gyro EKF (fusion.launch.py) and slam_toolbox
# online-async mapping seeded by the fused odom.
#
# Run alongside the drive stack (gamepad.launch.py), e.g.:
#   source ~/dean/rs2_ws/install/setup.bash
#   ros2 launch sentinel_bringup mapping.launch.py
#   ros2 launch sentinel_drive_cpp gamepad.launch.py
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SHARE = get_package_share_directory('sentinel_bringup')


def generate_launch_description():
    # slam:=false keeps lidar + EKF (localization needs the same
    # /scan + odom -> base_link feeds as mapping) without slam_toolbox.
    slam = LaunchConfiguration('slam', default='true')
    declare_slam = DeclareLaunchArgument('slam', default_value=slam)

    # Wheel odom + gyro EKF: publishes /wheel_imu_odom_filtered and
    # odom -> base_link, the TF chain SLAM registers scans against.
    # Its SetEnvironmentVariable (RSUSB lib path) applies launch-wide.
    fusion = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(SHARE, 'launch', 'fusion.launch.py')))

    # RPLIDAR A1 over the CP210x serial adapter (115200 baud).
    # Started 4 s late: starting it concurrently with the realsense
    # driver's USB init killed the SDK with SIGABRT twice (Orin Nano,
    # both devices on one USB controller).
    rplidar = TimerAction(
        period=4.0,
        actions=[Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': '/dev/sentinel_lidar',
                'serial_baudrate': 115200,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
            }],
            output='screen',
        )],
    )

    # Lidar extrinsics: 0.1524 m (6 in) forward of base_link along +X,
    # 0.1016 m (4 in) above it, sensor X facing -X of robot travel
    # (yaw pi).
    laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '0.1524', '--y', '0', '--z', '0.1016',
                   '--roll', '0', '--pitch', '0', '--yaw', '3.14159265',
                   '--frame-id', 'base_link',
                   '--child-frame-id', 'laser'],
    )

    slam_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'),
                         'launch', 'online_async_launch.py')),
        launch_arguments={
            'slam_params_file': os.path.join(
                SHARE, 'config', 'slam_params_online_async.yaml')}.items(),
        condition=IfCondition(slam),
    )

    return LaunchDescription(
        [declare_slam, fusion, rplidar, laser_tf, slam_include])
