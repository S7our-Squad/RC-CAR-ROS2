from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Create a dummy base_link frame for the robot
    If your car has its own TF broadcaster, skip this
    """
    
    tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '0.0', '0.0', '0.0',
            '0.0', '0.0', '0.0',
            'map',
            'base_link'
        ]
    )
    
    return LaunchDescription([tf_publisher])
