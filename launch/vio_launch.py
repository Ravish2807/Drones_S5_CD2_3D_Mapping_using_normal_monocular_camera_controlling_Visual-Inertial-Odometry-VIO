import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('vio_estimator')
    rviz_config_file = os.path.join(pkg_share, 'config', 'vio_rviz.rviz')
    if not os.path.exists(rviz_config_file):
        rviz_config_file = os.path.join(pkg_share, 'vio_rviz.rviz')

    vio_node = Node(
        package='vio_estimator',
        executable='vio_node',
        name='vio_estimator_node',
        output='screen',
        parameters=[{
            'config_path': os.path.join(pkg_share, 'config', 'project_vio.yaml')
        }]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        vio_node,
        rviz_node
    ])
