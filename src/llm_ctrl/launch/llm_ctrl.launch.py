from launch import LaunchDescription
from launch_ros.actions import Node


from launch.actions import OpaqueFunction

def launch_if_not_running(context):
    import subprocess

    try:
        output = subprocess.check_output(
            ['ros2', 'node', 'list'],
            text=True
        )
    except Exception:
        output = ""

    if '/motor_signals_node' in output:
        print('[launch] motor_signals_node already running → skipping launch')
        return [Node(
            package='llm_ctrl',
            executable='json_to_ackermann',
            name='json_to_ackermann_node',
            output='screen'
        )]

    print('[launch] motor_signals_node not running → launching')
    return [
        Node(
            package='llm_ctrl',
            executable='json_to_ackermann',
            name='json_to_ackermann_node',
            output='screen'
        ),
        Node(
            package='manual_ctrl',
            executable='motor_signals',
            name='motor_signals_node',
            output='screen'
        )
    ]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_if_not_running),
    ])
