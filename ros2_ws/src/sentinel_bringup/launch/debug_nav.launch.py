# Navigation debug companion. Run alongside a live nav2 stack:
#
#   ros2 launch sentinel_bringup debug_nav.launch.py
#
# Starts a curated ros2 bag recorder and a one-shot node that flips the
# key nav2 nodes to 'debug' logging. Bag lands in ~/bags/nav_debug_<ts>.
import os
import time

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

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

    logger = Node(
        package='sentinel_bringup',
        executable='debug_logger',
        name='debug_logger',
        output='screen',
    )

    return LaunchDescription([bag, logger])
