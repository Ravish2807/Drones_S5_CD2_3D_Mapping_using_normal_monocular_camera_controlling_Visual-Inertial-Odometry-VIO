import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('drones_controller')

    controller_config = os.path.join(pkg_share, 'config', 'controller.yaml')
    vehicle_config = os.path.join(pkg_share, 'config', 'vehicle.yaml')

    enable_command_arg = DeclareLaunchArgument(
        'enable_command',
        default_value='true',
        description='Whether to enable publishing control commands to /ap/v1/cmd_vel'
    )

    controller_node = Node(
        package='drones_controller',
        executable='controller_node',
        name='so3_controller',
        output='screen',
        parameters=[
            controller_config,
            vehicle_config,
            {'safety.enable_command': LaunchConfiguration('enable_command')}
        ]
    )

    return LaunchDescription([
        enable_command_arg,
        controller_node
    ])
