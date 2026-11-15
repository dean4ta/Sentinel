# Goal stress test: /goal_pose failure modes vs healthy control.
# Run alongside the live bringup (nav2 stack already up):
#
#   ros2 launch sentinel_debug goal_stress_test.launch.py
#
# DEFAULT run is HEALTHY: goal 12 m ahead of the robot's CURRENT amcl
# pose (map frame, current heading, fresh stamp) — the robot really
# drives 12 m forward. Clear the space and stand behind.
#
# CLI overrides (launch arg names use underscores, not dashes):
#   frame:=map              goal header frame       (default map)
#   truncate_stamp:=false   fresh stamp             (default false)
#   duration:=60.0          run length in seconds   (default 60.0)
#   distance:=12.0          goal distance in meters (default 12.0)
#
# Foxglove mimic (the bug reproduction — expect FAST failure, not the
# 10 s buffer death: the truncated whole-second stamp is up to ~1 s old,
# past planner_server's 0.2 s transform_tolerance, so the first plan
# attempt raises an Extrapolation Error and recovery triggers
# immediately):
#   ros2 launch sentinel_debug goal_stress_test.launch.py \
#       frame:=base_link truncate_stamp:=true
#
# Isolated ~10 s TF-buffer death (base_link goal with a FRESH stamp;
# each ~1 s replan re-transforms the frozen stamp until it falls out of
# the TF buffer after ~10 s):
#   ros2 launch sentinel_debug goal_stress_test.launch.py \
#       frame:=base_link truncate_stamp:=false
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    frame = LaunchConfiguration('frame', default='map')
    truncate_stamp = LaunchConfiguration('truncate_stamp', default='false')
    duration = LaunchConfiguration('duration', default='60.0')
    distance = LaunchConfiguration('distance', default='12.0')

    return LaunchDescription([
        DeclareLaunchArgument('frame', default_value='map'),
        DeclareLaunchArgument('truncate_stamp', default_value='false'),
        DeclareLaunchArgument('duration', default_value='60.0'),
        DeclareLaunchArgument('distance', default_value='12.0'),
        Node(
            package='sentinel_debug',
            executable='goal_stress_test',
            name='goal_stress_test',
            output='screen',
            parameters=[{
                'frame': frame,
                'truncate_stamp': truncate_stamp,
                'duration': duration,
                'distance': distance,
            }],
        ),
    ])
