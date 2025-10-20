#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get package directories
    pkg_dir = get_package_share_directory('turtlebot3_autonav')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_dir, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use'
    )
    
    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )
    
    use_composition_arg = DeclareLaunchArgument(
        'use_composition',
        default_value='false',
        description='Use composed bringup if true'
    )
    
    map_file_arg = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Full path to map yaml file to load'
    )
    
    # Get configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    
    # Configure parameters
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites={},
        convert_types=True
    )
    
    # Include Nav2 bringup launch
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': configured_params,
            'autostart': autostart,
            'use_composition': use_composition,
            'map': LaunchConfiguration('map'),
        }.items()
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        params_file_arg,
        autostart_arg,
        use_composition_arg,
        map_file_arg,
        nav2_bringup_launch
    ])

