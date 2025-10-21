#!/usr/bin/env python3

"""
Simplified Gazebo-only launch file for testing
Launches only Gazebo with the maze world and TurtleBot3
"""

import os
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('turtlebot3_autonav')
    
    # Paths
    world_file = os.path.join(pkg_dir, 'world', 'maze.world')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'turtlebot3_burger.urdf.xacro')
    
    # Process URDF with xacro
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )
    
    # Start Gazebo
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', world_file, '-r'],
        output='screen'
    )
    
    # Spawn robot at center of maze - delayed to ensure robot_state_publisher is ready
    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-name', 'turtlebot3_burger',
                    '-topic', 'robot_description',
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.01'
                ],
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot
    ])

