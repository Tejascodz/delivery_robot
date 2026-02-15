#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_core = get_package_share_directory('delivery_robot_core')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Delivery Manager Node
    delivery_manager_node = Node(
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
    
    # RViz2 with delivery config
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_core, 'config', 'delivery_display.rviz')],
        condition=launch.conditions.IfCondition(LaunchConfiguration('use_rviz'))
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                            description='Use simulation time if true'),
        DeclareLaunchArgument('use_rviz', default_value='true',
                            description='Launch RViz2'),
        
        delivery_manager_node,
        rviz2
    ])
