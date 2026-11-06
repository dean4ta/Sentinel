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
from launch_ros.actions import Node

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

    # Nav2 node log level: log_level:=debug for planner/controller/bt/
    # behavior debug output (see navigation.launch.py).
    log_level = LaunchConfiguration('log_level', default='info')
    declare_log_level = DeclareLaunchArgument(
        'log_level', default_value=log_level)

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(SHARE, 'launch', 'navigation.launch.py')),
        launch_arguments={'log_level': log_level}.items(),
        condition=IfCondition(nav),
    )

    # Programmer notes logger: take-notes:=true also starts the note
    # logger; pair it with `ros2 run sentinel_bringup note_taker`.
    take_notes = LaunchConfiguration('take-notes', default='false')
    declare_take_notes = DeclareLaunchArgument(
        'take-notes', default_value=take_notes)
    notes = Node(
        package='sentinel_bringup',
        executable='programmer_notation',
        name='programmer_notation',
        output='screen',
        condition=IfCondition(take_notes),
    )

    # Foxglove bridge on :9090. Own node defs instead of the stock
    # include: output='log' sends the "write queue full" chatter to
    # ~/.ros/log instead of the console, and write_queue_size is raised
    # above the 1000 default so a slow client (Foxglove over wifi) drops
    # fewer messages on bursts.
    rosbridge = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='log',
        parameters=[{
            'port': 9090,
            'write_queue_size': 10000,
            'max_message_size': 10000000,
        }],
    )
    rosapi = Node(
        package='rosapi',
        executable='rosapi_node',
        name='rosapi',
        output='log',
    )

    # Arm bridge: joint feedback + moves to the saved home pose after
    # the ESP32 boot drain. Home is end-effector XYZ (mm) + gripper rad,
    # captured by hand with torque off (2026-10-23).
    arm = Node(
        package='sentinel_arm',
        executable='arm_bridge',
        name='arm_bridge',
        output='screen',
        parameters=[{
            'home': [3.28, -47.44, 69.38, 3.134],
            'spd': 200,
            'acc': 40,
        }],
    )

    return LaunchDescription([
        declare_mapping,
        declare_localize,
        declare_nav,
        declare_log_level,
        declare_take_notes,
        drive,
        map_stack,
        localize_feeds,
        localization,
        navigation,
        notes,
        arm,
        rosbridge,
        rosapi,
    ])
