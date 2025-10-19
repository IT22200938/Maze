#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directories
    pkg_turtlebot3_maze = get_package_share_directory('turtlebot3_maze_navigation')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    # Paths
    world_file = os.path.join(pkg_turtlebot3_maze, 'worlds', 'maze_world.sdf')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='-1.25')
    y_pose = LaunchConfiguration('y_pose', default='1.25')
    z_pose = LaunchConfiguration('z_pose', default='0.01')
    
    # Set TurtleBot3 model
    os.environ['TURTLEBOT3_MODEL'] = 'burger'
    
    # Gazebo launch
    gzserver_cmd = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-s', world_file],
        output='screen'
    )
    
    # Robot State Publisher
    urdf_file = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'urdf',
        'turtlebot3_burger.urdf'
    )
    
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc
        }]
    )
    
    # Spawn TurtleBot3
    spawn_turtlebot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'turtlebot3',
            '-file', urdf_file,
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose
        ],
        output='screen'
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                            description='Use simulation time'),
        DeclareLaunchArgument('x_pose', default_value='-1.25',
                            description='X position of robot'),
        DeclareLaunchArgument('y_pose', default_value='1.25',
                            description='Y position of robot'),
        DeclareLaunchArgument('z_pose', default_value='0.01',
                            description='Z position of robot'),
        gzserver_cmd,
        robot_state_publisher,
        spawn_turtlebot
    ])

