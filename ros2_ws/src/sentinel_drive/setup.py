from setuptools import find_packages, setup

package_name = 'sentinel_drive'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/gamepad.launch.py', 'launch/step_test.launch.py',
             'launch/fusion.launch.py', 'launch/mapping.launch.py',
             'launch/sentinel_bringup.launch.py',
             'launch/localization.launch.py',
             'launch/navigation.launch.py']),
        ('share/' + package_name + '/config',
            ['config/drive_params.yaml', 'config/ekf.yaml',
             'config/slam_params_online_async.yaml',
             'config/amcl_params.yaml',
             'config/nav2_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dean Fortier',
    maintainer_email='dean@example.com',
    description='Gamepad to Pico serial drive control for Sentinel 1.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'gamepad_twist = sentinel_drive.gamepad_twist:main',
            'cmd_vel_serial = sentinel_drive.cmd_vel_serial:main',
            'wheel_debug = sentinel_drive.wheel_debug:main',
            'speed_step_test = sentinel_drive.speed_step_test:main',
            'wheel_slip_detector = sentinel_drive.wheel_slip_detector:main',
            'wheel_odometry = sentinel_drive.wheel_odometry:main',
        ],
    },
)
