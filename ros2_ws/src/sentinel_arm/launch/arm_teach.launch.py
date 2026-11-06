# Arm bridge + teach-and-repeat node.
#
#   ros2 launch sentinel_arm arm_teach.launch.py
#
# Teach flow (manual): torque off -> hand-guide arm -> record -> repeat
#   ros2 service call arm/torque sentinel_msgs/srv/SetTorque "{enable: false}"
#   ros2 service call arm/record_waypoint sentinel_msgs/srv/RecordWaypoint "{name: 'pose1'}"
#   ros2 service call arm/torque sentinel_msgs/srv/SetTorque "{enable: true}"
# Save / load / play:
#   ros2 service call arm/waypoint_cmd sentinel_msgs/srv/WaypointCmd "{cmd: 'save', arg: 'demo'}"
#   ros2 service call arm/playback sentinel_msgs/srv/Playback "{cmd: 'start', filename: 'demo'}"
# Watch progress: ros2 topic echo arm/teach_status
#
# spd/acc deliberately low (50/10) for first teach/repeat tests; raise
# once replay looks good. No home move on launch - the arm stays put.
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
    teach = Node(
        package='sentinel_arm',
        executable='arm_teach',
        name='arm_teach',
        output='screen',
        parameters=[{
            'waypoint_dir': '~/arm_waypoints',
            'reach_tol': 0.02,
            'hold_samples': 2,
            'waypoint_timeout': 10.0,
        }],
    )
    return LaunchDescription([bridge, teach])
