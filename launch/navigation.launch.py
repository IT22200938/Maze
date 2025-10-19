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
    pkg_turtlebot3_maze = get_package_share_directory('turtlebot3_maze_navigation')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_file = LaunchConfiguration('map', 
                                   default=os.path.join(pkg_turtlebot3_maze, 
                                                       'maps', 
                                                       'maze_map.yaml'))
    params_file = LaunchConfiguration('params_file',
                                     default=os.path.join(pkg_turtlebot3_maze,
                                                         'config',
                                                         'nav2_params.yaml'))
    
    # Nav2 bringup
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file
        }.items()
    )
    
    # RViz
    rviz_config_file = os.path.join(pkg_turtlebot3_maze, 'config', 'navigation.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                            description='Use simulation time'),
        DeclareLaunchArgument('map', default_value=map_file,
                            description='Full path to map yaml file'),
        DeclareLaunchArgument('params_file', default_value=params_file,
                            description='Full path to nav2 params file'),
        nav2_bringup_launch,
        rviz_node
    ])

