from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy_linux',
            executable='joy_linux_node',
            name='joy_linux_node', 
        ),
        Node(
            package='joy_controller',
            executable='joy_controller_node',
            name='joy_controller_node', 
        )
    ])
