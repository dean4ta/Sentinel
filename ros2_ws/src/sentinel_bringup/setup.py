from setuptools import find_packages, setup

package_name = 'sentinel_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/fusion.launch.py', 'launch/mapping.launch.py',
             'launch/sentinel_bringup.launch.py',
             'launch/localization.launch.py',
             'launch/navigation.launch.py',
             'launch/usb_interface_test.launch.py',
             'launch/debug_nav.launch.py']),
        ('share/' + package_name + '/config',
            ['config/drive_params.yaml', 'config/ekf.yaml',
             'config/slam_params_online_async.yaml',
             'config/amcl_nav2_params.yaml']),
        ('share/' + package_name + '/udev',
            ['config/udev/99-sentinel-usb.rules']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dean Fortier',
    maintainer_email='dean@example.com',
    description='Gamepad to Pico serial drive control for Sentinel 1.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'gamepad_twist = sentinel_bringup.gamepad_twist:main',
            'wheel_debug = sentinel_bringup.wheel_debug:main',
            'speed_step_test = sentinel_bringup.speed_step_test:main',
            'publish_initial_pose = sentinel_bringup.publish_initial_pose:main',
            'sensor_check = sentinel_bringup.sensor_check:main',
            'note_taker = sentinel_bringup.note_taker:main',
            'programmer_notation = sentinel_bringup.programmer_notation:main',
            'debug_logger = sentinel_bringup.debug_logger:main',
        ],
    },
)
