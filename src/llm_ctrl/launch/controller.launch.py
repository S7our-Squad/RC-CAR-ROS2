from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='llm_ctrl',
            executable='controller',
            name='controller_node',
            output='screen'
        ),
    ])
