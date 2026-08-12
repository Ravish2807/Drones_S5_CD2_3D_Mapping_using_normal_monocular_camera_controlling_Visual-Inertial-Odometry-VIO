from setuptools import setup

package_name = 'drones_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
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
            'share/' + package_name + '/config',
            ['config/controller.yaml']
        ),
    ],
    install_requires=[
        'setuptools',
        'numpy',
    ],
    zip_safe=True,
    description='SO(3) controller for ArduPilot SITL + Gazebo',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'controller_node = drones_controller.controller_node:main',
            'teleop = drones_controller.teleop:main',
        ],
    },
)
