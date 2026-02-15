#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directories
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    pkg_navigation = get_package_share_directory('delivery_robot_navigation')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_yaml_file = LaunchConfiguration('map', default=os.path.join(pkg_navigation, 'maps', 'map.yaml'))
    params_file = LaunchConfiguration('params', default=os.path.join(pkg_navigation, 'config', 'nav2_params.yaml'))
    
    # Include Nav2 bringup
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml_file,
            'params_file': params_file
        }.items()
    )
    
    # Lifecycle manager for Nav2 nodes
    lifecycle_nodes = ['controller_server',
                       'planner_server',
                       'behavior_server',
                       'bt_navigator',
                       'waypoint_follower',
                       'velocity_smoother']
    
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'autostart': True,
                     'node_names': lifecycle_nodes}]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                            description='Use simulation time if true'),
        DeclareLaunchArgument('map', default_value=map_yaml_file,
                            description='Full path to map yaml file'),
        DeclareLaunchArgument('params', default_value=params_file,
                            description='Full path to params file'),
        
        navigation_launch,
        lifecycle_manager
    ])
