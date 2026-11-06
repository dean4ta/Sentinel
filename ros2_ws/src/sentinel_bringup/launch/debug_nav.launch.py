# Navigation debug companion: curated ros2 bag recorder. Run alongside
# a live nav2 stack (started with log_level:=debug for node logs):
#
#   ros2 launch sentinel_bringup debug_nav.launch.py
#
# Bag lands in ~/bags/nav_debug_<timestamp>.
import os
import time

from launch import LaunchDescription
from launch.actions import ExecuteProcess

TOPICS = [
    '/tf', '/tf_static', '/odom', '/amcl_pose', '/scan', '/map',
    '/plan', '/local_plan', '/transformed_global_plan',
    '/cmd_vel', '/goal_pose', '/initialpose',
    '/navigate_to_pose/_action/feedback', '/navigate_to_pose/_action/status',
    '/behavior_tree_log',
    '/wheel_speeds', '/wheel_consistency', '/wheel_odom', '/encoder',
    '/local_costmap/costmap_raw', '/global_costmap/costmap_raw',
    '/received_global_plan', '/smooth_path',
    '/velocity_smoother/trajectory',
]


def generate_launch_description():
    out = os.path.expanduser(
        '~/bags/nav_debug_' + time.strftime('%Y%m%d_%H%M%S'))

    bag = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', out] + TOPICS,
        output='screen',
    )

    return LaunchDescription([bag])
