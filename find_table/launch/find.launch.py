from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    find_service_server = Node(
        package='find_table',
        executable='find_service_server_node',
        output='screen',
        name='find_service_server',
        emulate_tty=True,
        
    )

    return LaunchDescription(
        [
            find_service_server
        ]
    )