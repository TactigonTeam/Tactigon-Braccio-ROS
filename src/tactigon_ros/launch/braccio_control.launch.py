from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tactigon_ros',
            executable='tactigon_data',
            name='tactigon_data',
        ),
        Node(
            package='tactigon_ros',
            executable='braccio_control',
            name='braccio_control',
        ),
        Node(
            package='braccio_ros',
            executable='braccio_communication',
            name='braccio_communication',
        ),
    ])