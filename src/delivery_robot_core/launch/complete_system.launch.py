#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch.conditions import IfCondition

def generate_launch_description():
    # Get package directories
    pkg_description = get_package_share_directory('delivery_robot_description')
    pkg_navigation = get_package_share_directory('delivery_robot_navigation')
    pkg_perception = get_package_share_directory('delivery_robot_perception')
    pkg_core = get_package_share_directory('delivery_robot_core')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    use_navigation = LaunchConfiguration('use_navigation', default='true')
    use_perception = LaunchConfiguration('use_perception', default='true')
    map_file = LaunchConfiguration('map', default=PathJoinSubstitution([pkg_navigation, 'maps', 'map.yaml']))
    world_file = LaunchConfiguration('world', default=PathJoinSubstitution([pkg_description, 'worlds', 'delivery_world.world']))
    
    # Robot description
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_description, 'launch', 'display.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'gui': 'false'
        }.items()
    )
    
    # Navigation stack
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_navigation, 'launch', 'navigation.launch.py')
        ),
        condition=IfCondition(use_navigation),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_file,
            'params': PathJoinSubstitution([pkg_navigation, 'config', 'nav2_params.yaml'])
        }.items()
    )
    
    # Localization
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_navigation, 'launch', 'localization.launch.py')
        ),
        condition=IfCondition(use_navigation),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_file,
            'params': PathJoinSubstitution([pkg_navigation, 'config', 'nav2_params.yaml'])
        }.items()
    )
    
    # Perception nodes
    perception_group = GroupAction(
        condition=IfCondition(use_perception),
        actions=[
            PushRosNamespace('perception'),
            Node(
                package='delivery_robot_perception',
                executable='obstacle_detection',
                name='obstacle_detection',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'obstacle_distance_threshold': 1.0,
                    'cluster_tolerance': 0.2,
                    'min_cluster_size': 3,
                    'robot_radius': 0.4
                }]
            )
        ]
    )
    
    # Delivery manager
    delivery_manager = Node(
        package='delivery_robot_core',
        executable='delivery_manager',
        name='delivery_manager',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'wait_at_pickup_duration': 5.0,
            'wait_at_dropoff_duration': 3.0,
            'task_timeout': 300.0,
            'max_retries': 3,
            'base_location': 'base',
            'base_pose.x': 0.0,
            'base_pose.y': 0.0,
            'base_pose.z': 0.0,
            'base_pose.yaw': 0.0
        }]
    )
    
    # RViz2
    rviz_config = PathJoinSubstitution([pkg_core, 'config', 'delivery_display.rviz'])
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz)
    )
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument('use_sim_time', default_value='true',
                            description='Use simulation time if true'),
        DeclareLaunchArgument('use_rviz', default_value='true',
                            description='Launch RViz2'),
        DeclareLaunchArgument('use_navigation', default_value='true',
                            description='Launch navigation stack'),
        DeclareLaunchArgument('use_perception', default_value='true',
                            description='Launch perception nodes'),
        DeclareLaunchArgument('map', default_value=map_file,
                            description='Full path to map file'),
        DeclareLaunchArgument('world', default_value=world_file,
                            description='Full path to world file'),
        
        # Launch all components
        robot_description_launch,
        navigation_launch,
        localization_launch,
        perception_group,
        delivery_manager,
        rviz2
    ])
