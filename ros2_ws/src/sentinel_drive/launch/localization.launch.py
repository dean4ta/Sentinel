# AMCL localization: map_server serving the saved occupancy map + amcl
# localizing against it with /scan and the fused wheel+gyro odom.
# amcl publishes map -> odom and /amcl_pose.
#
# Run alongside the drive stack (sentinel_bringup without mapping):
#   ros2 launch sentinel_drive sentinel_bringup.launch.py
#   ros2 launch sentinel_drive localization.launch.py
#
# Then give AMCL its start pose in Foxglove (3D panel, right-click ->
# Publish pose estimate) or RViz (2D Pose Estimate) - /initialpose.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SHARE = get_package_share_directory('sentinel_drive')


def generate_launch_description():
    map_file = LaunchConfiguration(
        'map_file', default='/home/sentinel1/maps/robotics_lab.yaml')
    declare_map_file = DeclareLaunchArgument(
        'map_file', default_value=map_file)

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{'yaml_filename': map_file}],
        output='screen',
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        # Single source of AMCL config: acml_nav2_params.yaml (nav mode uses
        # the same file).
        parameters=[os.path.join(SHARE, 'config', 'acml_nav2_params.yaml')],
        output='screen',
    )

    # map_server and amcl are lifecycle nodes; autostart activates both.
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        parameters=[{
            'node_names': ['map_server', 'amcl'],
            'autostart': True,
            'use_sim_time': False,
        }],
        output='screen',
    )

    return LaunchDescription([
        declare_map_file, map_server, amcl, lifecycle_manager])
