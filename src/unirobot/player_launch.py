from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='unirobot',
            executable='unirobot',
            arguments=['unirobot_1'],
            output='screen'
        ),
        Node(
            package='unirobot',
            executable='unirobot',
            arguments=['unirobot_2'],
            output='screen'
        )
    ])

