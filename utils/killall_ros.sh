#!/bin/bash
# Kill every Sentinel ROS2 node after a run. Run on the Orin (or over ssh).
# SIGINT first (clean shutdown), then SIGKILL for stragglers that ignore it
# (nav2 lifecycle nodes, realsense driver).
#
# Usage: ./utils/killall_ros.sh

PATTERN="cmd_vel_serial|gamepad_twist|wheel_slip|wheel_odom|joy_nod[e]|rosbridge|rosapi|realsense2_camera_nod[e]|ekf_nod[e]|rplidar_nod[e]|static_transform_publishe[r]|slam_toolbox|map_server|amcl|lifecycle_manager|nav2|bt_navigator|planner_serv|controller_serv|behavior_serv|waypoint_follow|velocity_smooth|transform_listener|ros2 bag"

pkill -INT -f "$PATTERN" 2>/dev/null
sleep 3   # rosbag finalizes metadata.yaml on SIGINT
pkill -9 -f "$PATTERN" 2>/dev/null
sleep 1

echo "remaining ROS nodes:"
source /opt/ros/humble/setup.bash 2>/dev/null
ros2 node list 2>/dev/null || echo "(none)"
