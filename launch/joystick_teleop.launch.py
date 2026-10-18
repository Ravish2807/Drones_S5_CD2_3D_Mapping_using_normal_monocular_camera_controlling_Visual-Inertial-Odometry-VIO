from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    joystick_node = Node(
        package='drones_controller',
        executable='joystick_teleop',
        name='joystick_teleop',
        output='screen',
        parameters=[
            {'device': 'auto'},
            {'speed_normal': 1.0},
            {'speed_precision': 0.5},
            {'speed_turbo': 2.0},
            {'deadzone': 0.12},
            {'rate': 50.0}
        ]
    )

    return LaunchDescription([
        joystick_node
    ])
