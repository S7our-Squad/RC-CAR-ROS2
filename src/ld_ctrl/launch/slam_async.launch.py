
from launch import LaunchDescription
from launch_ros.actions import Node, IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """
    Launch SLAM_Toolbox with YDLIDAR X2
    Creates a real-time map of your environment
    """
    
    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Get SLAM configuration file
    slam_config = PathJoinSubstitution([
        FindPackageShare('ld_ctrl'),
        'config',
        'slam_toolbox_async.yaml'
    ])
    
    ld = LaunchDescription()
    
    # ============================================
    # 1. Launch LiDAR Driver
    # ============================================
    ld_ctrl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ld_ctrl'),
                'launch',
                'ld_ctrl.launch.py'
            ])
        )
    )
    ld.add_action(ld_ctrl_launch)
    
    # ============================================
    # 2. Static Transform (LiDAR to base_link)
    # ============================================
    # This tells ROS where the LiDAR sits on the car
    tf_static_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='log',
        arguments=[
            '0.0', '0.0', '0.15',  # Position: x, y, z (meters)
            '0.0', '0.0', '0.0',   # Rotation: roll, pitch, yaw (radians)
            'base_link',            # Parent frame
            'laser_frame'           # Child frame (from ydlidar.yaml)
        ]
    )
    ld.add_action(tf_static_publisher)
    
    # ============================================
    # 3. SLAM_Toolbox Node (Async Mode)
    # ============================================
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config],
        remappings=[
            ('/scan', '/scan'),
        ]
    )
    ld.add_action(slam_node)
    
    return ld
