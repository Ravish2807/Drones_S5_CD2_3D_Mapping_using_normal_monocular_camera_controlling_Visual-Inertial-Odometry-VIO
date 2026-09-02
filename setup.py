import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'vio_estimator'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(include=['core*', 'interfaces*', 'src*', 'ros2_nodes*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name] if os.path.exists('resource/' + package_name) else []),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml') + glob('config/*.rviz')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ravish',
    maintainer_email='ravish@example.com',
    description='Standalone MSCKF Visual-Inertial Odometry state estimator for ROS 2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vio_node = ros2_nodes.vio_node:main',
        ],
    },
)
