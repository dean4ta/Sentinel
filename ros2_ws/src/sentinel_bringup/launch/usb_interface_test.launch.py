# USB topology sensor test: one-command bringup of every currently
# visible sensor plus the sensor_check node, which samples each stream
# for a few seconds, prints a PASS/FAIL table, and exits - its exit
# shuts the whole launch down.
#
#   source ~/dean/rs2_ws/install/setup.bash   # source-built realsense2_camera
#   ros2 launch sentinel_bringup usb_interface_test.launch.py
#
# Starts: rplidar driver (/scan), realsense gyro-only, cmd_vel_serial
# (pico gateway, idles neutral without /cmd_vel), sensor_check (webcam
# is probed inside it directly via V4L2).
import os

from launch import LaunchDescription
from launch.actions import (RegisterEventHandler, SetEnvironmentVariable,
                            Shutdown, TimerAction)
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

SHARE = FindPackageShare('sentinel_bringup')

# RSUSB-built librealsense (source build in ~/dean/rs2_local) must load
# ahead of the ROS deb's lib (both are librealsense2.so.2). Same trick
# as fusion.launch.py.
RS2_LIB = '/home/sentinel1/dean/rs2_local/lib'


def generate_launch_description():
    env = SetEnvironmentVariable(
        'LD_LIBRARY_PATH',
        RS2_LIB + ':' + os.environ.get('LD_LIBRARY_PATH', ''))

    # D435i as pure gyro (same params as fusion.launch.py).
    realsense = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense2_camera',
        namespace='camera',
        parameters=[{
            'enable_depth': False,
            'enable_color': False,
            'enable_infra1': False,
            'enable_infra2': False,
            'enable_gyro': True,
            'enable_accel': False,
            'unite_imu_method': 0,
            'publish_tf': True,
            'angular_velocity_cov': 0.0001,
        }],
        output='screen',
    )

    # RPLIDAR A1 over the CP210x adapter. Delayed 4 s like
    # mapping.launch.py: concurrent USB init with the realsense driver
    # SIGABRT'd the SDK twice on this Orin.
    rplidar = TimerAction(
        period=4.0,
        actions=[Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': '/dev/sentinel_lidar',
                'serial_baudrate': 115200,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
            }],
            output='screen',
        )],
    )

    # Pico gateway: publishes /encoder from the USB serial line.
    serial_node = Node(
        package='sentinel_drive_cpp',
        executable='cmd_vel_serial',
        name='cmd_vel_serial',
        parameters=[[SHARE, '/config/drive_params.yaml']],
        output='screen',
    )

    # Checker runs after the sensors are up (lidar starts at t=4 s),
    # samples for 3 s, prints the table, exits -> launch shuts down.
    # The event handler targets the Node action itself (not the
    # TimerAction wrapper).
    check_action = Node(
        package='sentinel_bringup',
        executable='sensor_check',
        name='sensor_check',
        output='screen',
    )
    check_node = TimerAction(period=8.0, actions=[check_action])

    done = RegisterEventHandler(
        OnProcessExit(
            target_action=check_action,
            on_exit=[Shutdown()],
        )
    )

    return LaunchDescription([env, realsense, rplidar, serial_node,
                              check_node, done])
