# Wheel odom + D435i gyro fusion: realsense bringup, base_link ->
# camera_link static TF (camera mounted upside down: roll 180 deg),
# robot_localization EKF publishing /wheel_imu_odom_filtered and the
# odom -> base_link TF.
#
# Run alongside the drive stack (gamepad.launch.py or step_test), e.g.:
#   source ~/dean/rs2_ws/install/setup.bash   # source-built realsense2_camera
#   ros2 launch sentinel_drive fusion.launch.py &
#   ros2 launch sentinel_drive gamepad.launch.py
#
# The driver publishes camera_link -> camera_*_frame TFs; the EKF owns
# odom -> base_link. No other node may publish base_link.
import os

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

SHARE = get_package_share_directory('sentinel_drive')

# RSUSB-built librealsense (source build in ~/dean/rs2_local) must load
# ahead of the ROS deb's lib (both are librealsense2.so.2, ROS puts its
# own first in LD_LIBRARY_PATH). Kept as a constant: the path is
# Orin-specific, not a launch argument.
RS2_LIB = '/home/sentinel1/dean/rs2_local/lib'


def generate_launch_description():
    env = SetEnvironmentVariable(
        'LD_LIBRARY_PATH',
        RS2_LIB + ':' + os.environ.get('LD_LIBRARY_PATH', ''))

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--roll', '3.14159265', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'base_link',
                   '--child-frame-id', 'camera_link'],
    )

    # D435i as pure gyro: depth/color/infra/accel off (CPU), gyro on.
    # EKF uses raw gyro topic; united /imu sync path is flaky with the
    # RSUSB build (HARDWARE_CLOCK timestamps), don't enable it.
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
        }],
        output='screen',
    )

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[os.path.join(SHARE, 'config', 'ekf.yaml')],
        remappings=[('odometry/filtered', '/wheel_imu_odom_filtered')],
        output='screen',
    )

    return LaunchDescription([env, static_tf, realsense, ekf])
