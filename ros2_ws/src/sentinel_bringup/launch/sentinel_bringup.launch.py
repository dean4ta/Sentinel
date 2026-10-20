# One-shot bringup: drive stack + rosbridge (Foxglove), plus AMCL
# localization by default, Nav2 navigation with nav:=true, or the SLAM
# mapping stack when mapping:=true (localization is then auto-disabled:
# slam_toolbox owns map -> odom).
#
#   ros2 launch sentinel_bringup sentinel_bringup.launch.py              # drive + foxglove + localize
#   ros2 launch sentinel_bringup sentinel_bringup.launch.py nav:=true    # drive + foxglove + nav2
#   ros2 launch sentinel_bringup sentinel_bringup.launch.py localize:=false  # drive + foxglove
#   ros2 launch sentinel_bringup sentinel_bringup.launch.py mapping:=true  # drive + foxglove + slam
#
# Needs ~/dean/rs2_ws sourced whenever localization or mapping is on
# (both launch the realsense driver via fusion.launch.py); only
# localize:=false runs without it.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

SHARE = get_package_share_directory('sentinel_bringup')
# Drive stack nodes are C++ now: gamepad launch lives in sentinel_drive_cpp
# (gamepad_twist itself stays in this package).
CPP_SHARE = get_package_share_directory('sentinel_drive_cpp')


def generate_launch_description():
    mapping = LaunchConfiguration('mapping', default='false')
    declare_mapping = DeclareLaunchArgument('mapping', default_value=mapping)

    # Pico gateway, wheel odometry inputs, PS4 control.
    drive = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(CPP_SHARE, 'launch', 'gamepad.launch.py')))

    # Lidar, laser TF, gyro+wheel EKF, slam_toolbox (brings its own
    # fusion.launch.py include).
    map_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(SHARE, 'launch', 'mapping.launch.py')),
        condition=IfCondition(mapping),
    )

    # Nav2 stack (brings its own map_server + amcl). nav:=true disables
    # the plain localization stack below.
    nav = LaunchConfiguration('nav', default='true')
    declare_nav = DeclareLaunchArgument('nav', default_value=nav)

    # AMCL on the saved map: lidar + EKF feeds (mapping.launch.py with
    # slam:=false) plus map_server + amcl. Off while mapping (slam_toolbox
    # and amcl would both publish map -> odom) or nav (nav2 brings its own
    # map_server + amcl).
    localize = LaunchConfiguration('localize', default='true')
    declare_localize = DeclareLaunchArgument('localize', default_value=localize)
    localize_condition = IfCondition(PythonExpression(
        ['"', localize, '" == "true" and "', mapping, '" == "false" and "',
         nav, '" == "false"']))
    # The lidar + EKF feeds are shared: any non-mapping mode needs them
    # (nav2's amcl consumes /scan + odom -> base_link just like ours).
    feeds_condition = IfCondition(PythonExpression(
        ['"', mapping, '" == "false" and ("', localize,
         '" == "true" or "', nav, '" == "true")']))
    localize_feeds = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(SHARE, 'launch', 'mapping.launch.py')),
        launch_arguments={'slam': 'false'}.items(),
        condition=feeds_condition,
    )
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(SHARE, 'launch', 'localization.launch.py')),
        condition=localize_condition,
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(SHARE, 'launch', 'navigation.launch.py')),
        condition=IfCondition(nav),
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
        declare_localize,
        declare_nav,
        drive,
        map_stack,
        localize_feeds,
        localization,
        navigation,
        rosbridge,
    ])
