#!/usr/bin/env python3

'''
    Launches a Segmentation node.
'''
import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # Getting directories and launch-files
    segmentation_dir = get_package_share_directory('laser_segmentation')
    default_params_file = os.path.join(segmentation_dir, 'params', 'default_params.yml')
    rviz_config = os.path.join(get_package_share_directory('laser_segmentation'), 'rviz', 'config.rviz')

    # Input parameters declaration
    params_file = LaunchConfiguration('params_file')

    declare_params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters file with segmentation configuration'
    )

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
    #    'scan_topic': scan_topic, 
    #    'segments_topic': segments_topic, 
    #    'segmentation_type': segmentation_type
    }

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True
    )

    # Prepare the laser segmentation node.
    segmentation_node = Node(
        package = 'laser_segmentation',
        namespace = '',
        executable = 'laser_segmentation',
        name = 'segmentation',
        parameters=[configured_params],
        emulate_tty = True
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    return LaunchDescription([
        declare_params_file_arg,
        segmentation_node
        # rviz2_node
    ])