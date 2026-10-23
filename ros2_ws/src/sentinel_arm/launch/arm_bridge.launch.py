# Sentinel arm bridge: RoArm-M2-S serial <-> ROS.
#
#   ros2 launch sentinel_arm arm_bridge.launch.py
#
# Topics:
#   arm/joint_states  (sensor_msgs/JointState)  - pose feedback, ~10 Hz
#   arm/joint_goals   (sensor_msgs/JointState)  - send a move
#
# Joint names: base, shoulder, elbow, clamp (radians).
# Example goal:
#   ros2 topic pub arm/joint_goals sensor_msgs/msg/JointState \
#     "{name: [base], position: [0.3]}"
#
# Params (arm_bridge.launch.py): port (default /dev/sentinel_arm),
# baud (115200), spd (50 steps/s), acc (10), read_rate (10 Hz).
# Opening the port resets the arm's ESP32 - it runs its init sequence
# (~8 s) before the bridge starts commanding.
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bridge = Node(
        package='sentinel_arm',
        executable='arm_bridge',
        name='arm_bridge',
        output='screen',
        parameters=[{
            'port': '/dev/sentinel_arm',
            'baud': 115200,
            'spd': 50,
            'acc': 10,
            'read_rate': 10.0,
        }],
    )
    return LaunchDescription([bridge])
