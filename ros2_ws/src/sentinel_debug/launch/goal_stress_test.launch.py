# Goal stress test: Foxglove-mimic /goal_pose + bag + rosout monitor.
# Run alongside the live bringup (nav2 stack already up):
#
#   ros2 launch sentinel_debug goal_stress_test.launch.py
#
# Defaults reproduce the Foxglove click exactly (frame base_link,
# truncated whole-second stamp) — the run is expected to show the
# Extrapolation Error ~10 s after the goal is published.
#
# CLI overrides (launch arg names use underscores, not dashes):
#   frame:=map              goal header frame        (default base_link)
#   truncate_stamp:=false   use a fresh stamp        (default true)
#   duration:=60.0          run length in seconds    (default 60.0)
#   distance:=12.0          goal x offset in meters  (default 12.0)
#
# A/B healthy case: ros2 launch sentinel_debug goal_stress_test.launch.py \
#   frame:=map truncate_stamp:=false
# The robot really drives `distance` meters forward — clear the space.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    frame = LaunchConfiguration('frame', default='base_link')
    truncate_stamp = LaunchConfiguration('truncate_stamp', default='true')
    duration = LaunchConfiguration('duration', default='60.0')
    distance = LaunchConfiguration('distance', default='12.0')

    return LaunchDescription([
        DeclareLaunchArgument('frame', default_value='base_link'),
        DeclareLaunchArgument('truncate_stamp', default_value='true'),
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
