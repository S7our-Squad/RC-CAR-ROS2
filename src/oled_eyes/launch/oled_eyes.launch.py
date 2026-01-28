from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='oled_eyes',
            executable='oled_eyes',
            name='oled_eyes',
            output='screen'
        )
    ])
