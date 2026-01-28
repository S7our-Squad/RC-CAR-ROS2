from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['bash', '-c', 
                 'source /home/driftpilot/RC-CAR-ROS2/whisper/bin/activate && '
                 'python3 /home/driftpilot/RC-CAR-ROS2/src/llm_ctrl/llm_ctrl/input.py'],
            name='input_node',
            output='screen',
            emulate_tty=True
        ),
    ])
