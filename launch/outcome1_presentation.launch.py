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

    ply_path_arg = DeclareLaunchArgument(
        'ply_path',
        default_value='',
        description='Path to .ply point cloud file (optional)'
    )

    ply_publisher_node = Node(
        package='vio_estimator',
        executable='ply_map_publisher',
        name='ply_map_publisher_node',
        output='screen',
        parameters=[{
            'ply_path': LaunchConfiguration('ply_path'),
            'voxel_size': 0.008,
            'frame_id': 'world'
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
        ply_path_arg,
        ply_publisher_node,
        rviz_node
    ])
