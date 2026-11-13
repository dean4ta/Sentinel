from setuptools import find_packages, setup

package_name = 'sentinel_arm'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/arm_bridge.launch.py',
             'launch/arm_teach.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dean Fortier',
    maintainer_email='dean@example.com',
    description='RoArm-M2-S serial to ROS bridge for Sentinel 1.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'arm_bridge = sentinel_arm.arm_bridge:main',
            'grasp_demo = sentinel_arm.grasp_demo:main',
            'arm_teach = sentinel_arm.arm_teach:main',
            'arm_teach_console = sentinel_arm.arm_teach_console:main',
        ],
    },
)
