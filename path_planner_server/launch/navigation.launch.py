import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    controller_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'controller.yaml')
    bt_navigator_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'bt.yaml')
    planner_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'planner_server.yaml')
    recovery_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'recovery.yaml')
    nav2_yaml = os.path.join(get_package_share_directory('localization_server'), 'config', 'amcl_config.yaml')
    map_file = os.path.join(get_package_share_directory('map_server'), 'config', 'starbot_real_area.yaml')
    filters_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'filters.yaml')
    segmentation_yaml = os.path.join(get_package_share_directory('laser_segmentation'), 'params', 'default_params.yml')
    remappings = [('/cmd_vel', '/turtlebot_5/cmd_vel')]

    return LaunchDescription([     
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': False}, 
                        {'yaml_filename':map_file}]
        ),
            
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='filter_mask_server',
            output='screen',
            emulate_tty=True,
            parameters=[filters_yaml]),

        Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='costmap_filter_info_server',
            output='screen',
            emulate_tty=True,
            parameters=[filters_yaml]),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_yaml]
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[controller_yaml],
            remappings=remappings),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[planner_yaml]),
            
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            parameters=[recovery_yaml],
            remappings=remappings,
            output='screen'),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[bt_navigator_yaml]),
        
        Node(
            package='attach_table',
            executable='approach_service_server_node',
            output='screen',
            name='approach_service_server',
            emulate_tty=True),

        # Node(
        #     package = 'laser_segmentation',
        #     namespace = '',
        #     executable = 'laser_segmentation',
        #     name = 'segmentation',
        #     parameters=[segmentation_yaml],
        #     emulate_tty = True),


        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            parameters=[{'use_sim_time': False}],
            output='screen',
            arguments=['-d' + os.path.join(get_package_share_directory('path_planner_server'), 'rviz', 'config.rviz')]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            parameters=[{'autostart': True},
                        {'node_names': ['map_server',
                                        'amcl',
                                        'controller_server',
                                        'planner_server',
                                        'behavior_server',
                                        'bt_navigator',
                                        'filter_mask_server',
                                        'costmap_filter_info_server']}])
    ])