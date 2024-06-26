import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # controller_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'controller.yaml')
    # configured_params = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'nav2_params.yaml')
    controller_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'controller.yaml')
    planner_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'planner_server.yaml')
    recovery_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'recovery.yaml')
    bt_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'bt.yaml')


    default_bt_xml_path = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'navigate_w_replanning_and_recovery.xml')
    
    

    remappings = [('/cmd_vel', '/diffbot_base_controller/cmd_vel_unstamped')]

    return LaunchDescription([ 
        # Launches local costmap stuff also    
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[controller_yaml],
            remappings=remappings
        ),
        
        #  Launches global costmap stuff
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[planner_yaml]
        ),
         
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            parameters=[recovery_yaml],
            remappings=remappings,
            output='screen'
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[bt_yaml]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            parameters=[{'use_sim_time': True}],
            output='screen',
            arguments=['-d' + os.path.join(get_package_share_directory('path_planner_server'), 'rviz', 'config.rviz')]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_pathplanner',
            output='screen',
            parameters=[{'autostart': True},
                        {'node_names': ['controller_server', 'planner_server', 'behavior_server', 'bt_navigator']}]
        )
    ])