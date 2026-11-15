from setuptools import find_packages, setup

package_name = 'sentinel_debug'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/goal_stress_test.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dean Fortier',
    maintainer_email='dean@example.com',
    description='Debug tools: long-goal stress test (T011) + step '
                'recorder for controller-level step tests (T012).',
    license='MIT',
    entry_points={
        'console_scripts': [
            'goal_stress_test = sentinel_debug.goal_stress_test:main',
            'step_recorder = sentinel_debug.step_recorder:main',
        ],
    },
)
