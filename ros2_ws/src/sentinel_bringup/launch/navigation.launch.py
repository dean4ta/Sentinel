# Nav2 navigation on the saved robotics_lab map. Launches the stock
# nav2_bringup stack (map_server, amcl, planner, controller, smoother,
# bt_navigator, behavior/waypoint/velocity nodes + lifecycle managers)
# with our params (amcl_nav2_params.yaml: mecanum DWB+rotation shim, /scan,
# fused odom, omni motion model). Run alongside the drive stack:
#
#   source ~/dean/rs2_ws/install/setup.bash
#   ros2 launch sentinel_bringup sentinel_bringup.launch.py nav:=true
#
# or alongside an already-running bringup (localize:=false):
#   ros2 launch sentinel_bringup navigation.launch.py
#
# Goals: Foxglove 3D panel right-click -> Publish pose estimate ->
# topic /goal_pose, or RViz 2D Nav Goal. Initial pose first: /initialpose.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SHARE = get_package_share_directory('sentinel_bringup')


def generate_launch_description():
    # Passed through to nav2_bringup; log_level:=debug enables the
    # per-node debug output for planner/controller/bt/behavior/etc.
    log_level = LaunchConfiguration('log_level')
    declare_log_level = DeclareLaunchArgument(
        'log_level', default_value='info')

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'),
                         'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': '/home/sentinel1/maps/robotics_lab.yaml',
            'params_file': os.path.join(SHARE, 'config', 'amcl_nav2_params.yaml'),
            'use_sim_time': 'False',
            'autostart': 'True',
            # nav2_bringup's lowercase-'false' defaults crash its own
            # PythonExpression conditions (NameError) unless overridden.
            'use_namespace': 'False',
            'use_composition': 'False',
            'use_respawn': 'False',
            'slam': 'False',
            'log_level': log_level,
        }.items(),
    )

    # One-shot low-confidence initial pose (0, 0, 0) once AMCL is up.
    initial_pose = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='sentinel_bringup',
                executable='publish_initial_pose',
                name='initial_pose_pub',
                output='screen',
            ),
        ],
    )

    return LaunchDescription([declare_log_level, nav2, initial_pose])
