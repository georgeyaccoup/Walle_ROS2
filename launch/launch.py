from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='walle_ROS2',
            executable='main_controller_1',
            name='main_controller_1',
        ),
        Node(
            package='walle_ROS2',
            executable='main_controller_2',
            name='main_controller_2',
        ),
        Node(
            package='walle_ROS2',
            executable='voice_listener',
            name='voice_listener',
        ),
        Node(
            package='walle_ROS2',
            executable='chatgpt_processor',
            name='chatgpt_processor',
        ),
    ])
