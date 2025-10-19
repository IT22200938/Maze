#!/usr/bin/env python3

"""
Simplified Gazebo Launch for TurtleBot3 Maze
This uses the standard TurtleBot3 Gazebo launch approach
Use this if the main gazebo_maze.launch.py has issues
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Paths
    pkg_turtlebot3_maze = get_package_share_directory('turtlebot3_maze_navigation')
    world_file = os.path.join(pkg_turtlebot3_maze, 'worlds', 'maze_world.sdf')
    
    # Set environment
    os.environ['TURTLEBOT3_MODEL'] = 'burger'
    os.environ['GAZEBO_MODEL_PATH'] = os.path.join(pkg_turtlebot3_maze, 'models')
    
    # Launch configuration
    x_pose = LaunchConfiguration('x_pose', default='-1.25')
    y_pose = LaunchConfiguration('y_pose', default='1.25')
    
    # Launch Gazebo with world
    # Note: Using gz sim for Gazebo Harmonic
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen'
    )
    
    # Try to use TurtleBot3 spawn if available
    # You may need to adjust this based on your TurtleBot3 installation
    try:
        pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
        
        # Include TurtleBot3 spawn launch
        turtlebot3_spawn = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('turtlebot3_gazebo'),
                    'launch',
                    'spawn_turtlebot3.launch.py'
                ])
            ]),
            launch_arguments={
                'x_pose': x_pose,
                'y_pose': y_pose,
            }.items()
        )
        
        return LaunchDescription([
            DeclareLaunchArgument('x_pose', default_value='-1.25'),
            DeclareLaunchArgument('y_pose', default_value='1.25'),
            gz_sim,
            turtlebot3_spawn
        ])
    except:
        # Fallback: just launch Gazebo
        print("TurtleBot3 Gazebo package not found. Launching only Gazebo.")
        print("You'll need to spawn the robot manually or use the main launch file.")
        
        return LaunchDescription([
            DeclareLaunchArgument('x_pose', default_value='-1.25'),
            DeclareLaunchArgument('y_pose', default_value='1.25'),
            gz_sim
        ])

