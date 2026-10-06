# One-shot bringup: drive stack + rosbridge (Foxglove), plus the SLAM
# mapping stack when mapping:=true.
#
#   ros2 launch sentinel_drive sentinel_bringup.launch.py              # drive + foxglove
#   ros2 launch sentinel_drive sentinel_bringup.launch.py mapping:=true  # + slam
#
# Needs ~/dean/rs2_ws sourced only when mapping:=true (realsense driver).
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

SHARE = get_package_share_directory('sentinel_drive')


def generate_launch_description():
    mapping = LaunchConfiguration('mapping', default='false')
    declare_mapping = DeclareLaunchArgument('mapping', default_value=mapping)

    # Pico gateway, wheel odometry inputs, PS4 control.
    drive = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(SHARE, 'launch', 'gamepad.launch.py')))

    # Lidar, laser TF, gyro+wheel EKF, slam_toolbox (brings its own
    # fusion.launch.py include).
    map_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(SHARE, 'launch', 'mapping.launch.py')),
        condition=IfCondition(mapping),
    )

    # Foxglove bridge on :9090.
    rosbridge = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rosbridge_server'),
                         'launch', 'rosbridge_websocket_launch.xml')),
        launch_arguments={'port': '9090'}.items(),
    )

    return LaunchDescription([
        declare_mapping,
        drive,
        map_stack,
        rosbridge,
    ])
