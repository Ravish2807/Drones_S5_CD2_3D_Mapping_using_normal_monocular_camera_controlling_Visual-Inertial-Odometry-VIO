import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('vio_estimator')
    rviz_config_file = os.path.join(pkg_share, 'config', 'vio_rviz.rviz')
    if not os.path.exists(rviz_config_file):
        rviz_config_file = os.path.join(pkg_share, 'vio_rviz.rviz')

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to launch RViz2 automatically'
    )

    dense_mapping_node = Node(
        package='vio_estimator',
        executable='dense_mapping_node',
        name='dense_mapping_node',
        output='screen',
        parameters=[{
            'config_path': os.path.join(pkg_share, 'config', 'project_vio.yaml'),
            'voxel_size': 0.010,
            'step': 2,
            'min_trans_m': 0.02,
            'min_rot_deg': 2.0,
            'rel_edge_grad': 0.08,
            'fx': 400.0,
            'fy': 400.0,
            'cx': 320.0,
            'cy': 240.0,
            'min_depth': 0.20,
            'max_depth': 4.50,
            'enable_bilateral': True,
            'enable_vibrance': True,
            'enable_obstacles': False
        }]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    return LaunchDescription([
        use_rviz_arg,
        dense_mapping_node,
        rviz_node
    ])
