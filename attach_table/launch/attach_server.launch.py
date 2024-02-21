from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    approach_service_server = Node(
        package='attach_table',
        executable='approach_service_server_node',
        output='screen',
        name='approach_service_server',
        emulate_tty=True,
        
    )

    return LaunchDescription(
        [
            approach_service_server
        ]
    )