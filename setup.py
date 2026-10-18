import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'drones_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')
        ),
    ],
    install_requires=[
        'setuptools',
        'numpy',
    ],
    zip_safe=True,
    description='Modular SO(3) closed-loop controller for ArduPilot SITL + Gazebo',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'controller_node = drones_controller.controller_node:main',
            'teleop = drones_controller.teleop:main',
            'cmd_vel_bridge = drones_controller.cmd_vel_bridge:main',
            'arm_and_takeoff = drones_controller.arm_and_takeoff:main',
            'joystick_teleop = drones_controller.joystick_teleop:main',
        ],
    },
)
