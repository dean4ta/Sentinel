# T007 grasp-demo bringup: Pico gateway (gripper pressure topic) +
# arm bridge + grasp_demo node.
#
#   ros2 launch sentinel_bringup grasp_demo.launch.py
#   ros2 service call /grasp/start std_srvs/srv/Trigger '{}'
#
# cmd_vel_serial runs only for its /gripper_pressure_reading publisher
# (10 Hz, raw gripper ADC) — no cmd_vel source is started, so the
# wheels never move. The arm bridge home is [0,0,0,0] = NO home move
# on boot: the arm stays put until the /grasp/start trigger (the demo's
# own final HOME stage returns it to the hand-captured home). Torque
# stays locked for the whole run.
#
# Fold-in note (for the eventual merge into sentinel_bringup.launch.py):
# the full bringup already starts arm_bridge with home
# [3.28, -47.44, 69.38, 3.134] — keep THAT home (or drop it; the demo's
# HOME stage returns the arm there anyway) and add the grasp_demo node
# plus (if not already running) cmd_vel_serial. Do not start a second
# arm_bridge or cmd_vel_serial: both open serial ports exclusively.
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pico = Node(
        package='sentinel_drive_cpp',
        executable='cmd_vel_serial',
        name='cmd_vel_serial',
        output='screen',
        parameters=[{
            'serial_port': '/dev/sentinel_pico',
        }],
    )
    bridge = Node(
        package='sentinel_arm',
        executable='arm_bridge',
        name='arm_bridge',
        output='screen',
        parameters=[{
            'port': '/dev/sentinel_arm',
            'baud': 115200,
            'spd': 200,
            'acc': 40,
            'read_rate': 10.0,
            'home': [0.0, 0.0, 0.0, 0.0],
        }],
    )
    demo = Node(
        package='sentinel_arm',
        executable='grasp_demo',
        name='grasp_demo',
        output='screen',
    )
    return LaunchDescription([pico, bridge, demo])
