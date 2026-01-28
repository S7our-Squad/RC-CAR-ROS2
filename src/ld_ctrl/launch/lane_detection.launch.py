from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """Launch lane detection pipeline (Ackermann output)"""
    
    lane_detector = Node(
        package='ld_ctrl',
        executable='lane_detector_node',
        name='lane_detector',
        output='screen',
        remappings=[
            ('/camera/image_raw', '/camera/image_raw')
        ]
    )
    
    lane_controller = Node(
        package='ld_ctrl',
        executable='lane_controller_node',
        name='lane_controller',
        output='screen',
        remappings=[
            ('/drive_ackermann', '/drive_ackermann')
        ]
    )
    
    return LaunchDescription([lane_detector, lane_controller])
