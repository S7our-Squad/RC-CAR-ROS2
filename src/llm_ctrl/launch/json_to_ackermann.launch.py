from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='llm_ctrl',
            executable='json_to_ackermann',
            name='json_to_ackermann_node',
            output='screen'
        ),
    ])

