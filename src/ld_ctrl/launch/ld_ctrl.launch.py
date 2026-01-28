from launch import LaunchDescription
from launch_ros.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    """
    Launch YDLiDAR X2 driver using official launch file
    - Includes RViz visualization
    - Uses your custom ydlidar.yaml config
    """
    
    # Get the YDLiDAR driver package location
    ydlidar_driver_share = FindPackageShare('ydlidar_ros2_driver')
    
    # Path to YOUR X2 config file
    config_file = PathJoinSubstitution([
        FindPackageShare('ld_ctrl'),  # YOUR ld_ctrl package
        'config',
        'ydlidar.yaml'
    ])
    
    # Include the OFFICIAL YDLiDAR launch file with YOUR config
    ydlidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                ydlidar_driver_share,
                'launch',
                'ydlidar_launch_view.py'  # Official launch + RViz
            ])
        ),
        launch_arguments={
            'params_file': config_file  # Use YOUR config
        }.items()
    )
    
    return LaunchDescription([ydlidar_launch])
