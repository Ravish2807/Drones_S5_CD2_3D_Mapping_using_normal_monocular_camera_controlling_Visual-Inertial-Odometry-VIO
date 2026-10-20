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

    dense_mapping_node = Node(
        package='vio_estimator',
        executable='dense_mapping_node',
        name='dense_mapping_node',
        output='screen',
        parameters=[{
            'config_path': os.path.join(pkg_share, 'config', 'project_vio.yaml'),
            'voxel_size': 0.010,
            'step': 2,
            'min_trans_m': 0.025,
            'min_rot_deg': 2.5,
            'rel_edge_grad': 0.08,
            'fx': 400.0,
            'fy': 400.0,
            'min_depth': 0.20,
            'max_depth': 4.50,
            'enable_bilateral': True,
            'enable_vibrance': True
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
        dense_mapping_node,
        rviz_node
    ])
