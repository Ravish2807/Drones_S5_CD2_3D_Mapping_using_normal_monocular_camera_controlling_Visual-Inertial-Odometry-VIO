import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('drones_controller')

    vehicle_config = os.path.join(pkg_share, 'config', 'vehicle.yaml')
    model1_config = os.path.join(pkg_share, 'config', 'model1_so3.yaml')
    model2_config = os.path.join(pkg_share, 'config', 'model2_dmdc.yaml')

    model_arg = DeclareLaunchArgument(
        'model',
        default_value='1',
        description='Controller model: "1" (Geometric SO(3) + EKF) or "2" (DMDc + LQR/MPC)'
    )

    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='mpc',
        description='Model 2 optimal control mode: "mpc" or "lqr"'
    )

    is_model_1 = PythonExpression(["'", LaunchConfiguration('model'), "' == '1'"])
    is_model_2 = PythonExpression(["'", LaunchConfiguration('model'), "' == '2'"])

    # Model 1: Nonlinear Model-Based Estimation and Geometric SO(3) Control
    node_model1 = Node(
        package='drones_controller',
        executable='model1_so3_controller',
        name='model1_so3_controller',
        output='screen',
        parameters=[
            model1_config,
            vehicle_config
        ],
        condition=IfCondition(is_model_1)
    )

    # Model 2: Data-Driven Linear Model (DMDc) and Optimal Control (LQR / MPC)
    node_model2 = Node(
        package='drones_controller',
        executable='model2_dmdc_controller',
        name='model2_dmdc_controller',
        output='screen',
        parameters=[
            model2_config,
            vehicle_config,
            {'control_mode': LaunchConfiguration('mode')}
        ],
        condition=IfCondition(is_model_2)
    )

    return LaunchDescription([
        model_arg,
        mode_arg,
        node_model1,
        node_model2
    ])
