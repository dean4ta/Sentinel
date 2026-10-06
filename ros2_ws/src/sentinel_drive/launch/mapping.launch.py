# SLAM mapping stack: RPLIDAR A1 driver, base_link -> laser static TF
# (lidar mounted 4 in above base_link, its X facing -X of robot travel:
# yaw 180 deg), wheel-odom + gyro EKF (fusion.launch.py) and slam_toolbox
# online-async mapping seeded by the fused odom.
#
# Run alongside the drive stack (gamepad.launch.py), e.g.:
#   source ~/dean/rs2_ws/install/setup.bash
#   ros2 launch sentinel_drive mapping.launch.py
#   ros2 launch sentinel_drive gamepad.launch.py
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

SHARE = get_package_share_directory('sentinel_drive')


def generate_launch_description():
    # Wheel odom + gyro EKF: publishes /wheel_imu_odom_filtered and
    # odom -> base_link, the TF chain SLAM registers scans against.
    # Its SetEnvironmentVariable (RSUSB lib path) applies launch-wide.
    fusion = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(SHARE, 'launch', 'fusion.launch.py')))

    # RPLIDAR A1 over the CP210x serial adapter (115200 baud).
    rplidar = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
        }],
        output='screen',
    )

    # Lidar extrinsics: 0.1016 m (4 in) above base_link, yaw pi.
    laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '0', '--y', '0', '--z', '0.1016',
                   '--roll', '0', '--pitch', '0', '--yaw', '3.14159265',
                   '--frame-id', 'base_link',
                   '--child-frame-id', 'laser'],
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'),
                         'launch', 'online_async_launch.py')),
        launch_arguments={
            'slam_params_file': os.path.join(
                SHARE, 'config', 'slam_params_online_async.yaml')}.items(),
    )

    return LaunchDescription([fusion, rplidar, laser_tf, slam])
