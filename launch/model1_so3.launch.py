import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('drones_controller')

    controller_config = os.path.join(pkg_share, 'config', 'model1_so3.yaml')
    vehicle_config = os.path.join(pkg_share, 'config', 'vehicle.yaml')

    node = Node(
        package='drones_controller',
        executable='model1_so3_controller',
        name='model1_so3_controller',
        output='screen',
        parameters=[
            controller_config,
            vehicle_config
        ]
    )

    return LaunchDescription([
        node
    ])
