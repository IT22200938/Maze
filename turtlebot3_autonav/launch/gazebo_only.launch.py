#!/usr/bin/env python3

"""
Simplified Gazebo-only launch file for testing
Launches only Gazebo with the maze world and TurtleBot3
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('turtlebot3_autonav')
    
    # Paths
    world_file = os.path.join(pkg_dir, 'world', 'maze.world')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'turtlebot3_burger.urdf.xacro')
    
    # Process URDF with xacro
    robot_desc = ExecuteProcess(
        cmd=['xacro', urdf_file],
        output='screen'
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc.output,
            'use_sim_time': True
        }]
    )
    
    # Start Gazebo
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', world_file, '-r'],
        output='screen'
    )
    
    # Spawn robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'turtlebot3_burger',
            '-topic', '/robot_description',
            '-x', '0.5',
            '-y', '0.5',
            '-z', '0.01'
        ],
        output='screen'
    )
    
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot
    ])

