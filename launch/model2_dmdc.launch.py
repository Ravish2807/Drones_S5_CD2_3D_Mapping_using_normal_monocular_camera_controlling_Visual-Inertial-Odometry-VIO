import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('drones_controller')

    controller_config = os.path.join(pkg_share, 'config', 'model2_dmdc.yaml')
    vehicle_config = os.path.join(pkg_share, 'config', 'vehicle.yaml')

    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='mpc',
        description='Optimal control mode: "mpc" (finite horizon constrained) or "lqr" (steady-state DARE)'
    )

    node = Node(
        package='drones_controller',
        executable='model2_dmdc_controller',
        name='model2_dmdc_controller',
        output='screen',
        parameters=[
            controller_config,
            vehicle_config,
            {'control_mode': LaunchConfiguration('mode')}
        ]
    )

    return LaunchDescription([
        mode_arg,
        node
    ])
