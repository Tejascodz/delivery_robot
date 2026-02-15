#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('delivery_robot_description')
    
    # Paths
    urdf_file = os.path.join(pkg_share, 'urdf', 'delivery_robot.urdf')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': open(urdf_file).read(),
            'use_sim_time': use_sim_time
        }]
    )
    
    # Joint state publisher (GUI version for testing)
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    )
    
    # RViz2
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_share, 'config', 'display.rviz')]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                            description='Use simulation time if true'),
        DeclareLaunchArgument('gui', default_value='true',
                            description='Use joint state publisher GUI'),
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz2
    ])
