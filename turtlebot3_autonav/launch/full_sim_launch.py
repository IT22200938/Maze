#!/usr/bin/env python3

"""
Full Simulation Launch File
Launches Gazebo, TurtleBot3, SLAM, and Navigation stack
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    pkg_dir = get_package_share_directory('turtlebot3_autonav')
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )
    
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_dir, 'world', 'maze.world'),
        description='Full path to world file'
    )
    
    nav_mode_arg = DeclareLaunchArgument(
        'nav_mode',
        default_value='nav2',
        description='Navigation mode: nav2 or dqn'
    )
    
    use_slam_arg = DeclareLaunchArgument(
        'use_slam',
        default_value='true',
        description='Whether to run SLAM'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz2'
    )
    
    enable_safety_arg = DeclareLaunchArgument(
        'enable_safety',
        default_value='true',
        description='Enable obstacle monitor safety'
    )
    
    enable_recording_arg = DeclareLaunchArgument(
        'enable_recording',
        default_value='true',
        description='Enable performance recording'
    )
    
    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    world_file = LaunchConfiguration('world')
    nav_mode = LaunchConfiguration('nav_mode')
    use_slam = LaunchConfiguration('use_slam')
    use_rviz = LaunchConfiguration('use_rviz')
    enable_safety = LaunchConfiguration('enable_safety')
    enable_recording = LaunchConfiguration('enable_recording')
    
    # Robot description
    urdf_file = os.path.join(pkg_dir, 'urdf', 'turtlebot3_burger.urdf.xacro')
    
    # Process URDF
    robot_description_command = ExecuteProcess(
        cmd=['xacro', urdf_file],
        output='screen'
    )
    
    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': ExecuteProcess(
                cmd=['xacro', urdf_file],
                output='screen'
            ).output
        }]
    )
    
    # Start Gazebo Sim
    gazebo_server = ExecuteProcess(
        cmd=['gz', 'sim', world_file, '-r', '-s'],
        output='screen'
    )
    
    gazebo_client = ExecuteProcess(
        cmd=['gz', 'sim', '-g'],
        output='screen'
    )
    
    # Spawn robot in Gazebo
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
    
    # ROS-Gazebo bridge for topics
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # SLAM launch
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'online_slam.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(use_slam)
    )
    
    # Nav2 launch (only if nav_mode is nav2)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'nav2_bringup.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )
    
    # DQN agent node (only if nav_mode is dqn)
    dqn_node = Node(
        package='turtlebot3_autonav',
        executable='dqn_node.py',
        name='dqn_agent',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'training_mode': False,
        }]
    )
    
    # Goal clicker UI node
    goal_clicker_node = Node(
        package='turtlebot3_autonav',
        executable='goal_clicker_node.py',
        name='goal_clicker',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'nav_mode': nav_mode,
        }]
    )
    
    # Obstacle monitor (safety node)
    obstacle_monitor_node = Node(
        package='turtlebot3_autonav',
        executable='obstacle_monitor',
        name='obstacle_monitor',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'safety_distance': 0.18,
            'enable_safety': enable_safety,
        }],
        condition=IfCondition(enable_safety)
    )
    
    # Performance recorder node
    recorder_node = Node(
        package='turtlebot3_autonav',
        executable='recorder_node.py',
        name='performance_recorder',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'navigation_mode': nav_mode,
        }],
        condition=IfCondition(enable_recording)
    )
    
    # RViz2
    rviz_config = os.path.join(pkg_dir, 'rviz', 'navigation.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz)
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add arguments
    ld.add_action(use_sim_time_arg)
    ld.add_action(world_arg)
    ld.add_action(nav_mode_arg)
    ld.add_action(use_slam_arg)
    ld.add_action(use_rviz_arg)
    ld.add_action(enable_safety_arg)
    ld.add_action(enable_recording_arg)
    
    # Add Gazebo
    ld.add_action(gazebo_server)
    ld.add_action(TimerAction(period=3.0, actions=[gazebo_client]))
    
    # Add robot
    ld.add_action(robot_state_publisher_node)
    ld.add_action(TimerAction(period=5.0, actions=[spawn_robot]))
    ld.add_action(TimerAction(period=5.0, actions=[gz_bridge]))
    
    # Add SLAM
    ld.add_action(TimerAction(period=8.0, actions=[slam_launch]))
    
    # Add navigation (Nav2 or DQN based on mode)
    ld.add_action(TimerAction(period=10.0, actions=[nav2_launch]))
    # ld.add_action(TimerAction(period=10.0, actions=[dqn_node]))  # Uncomment for DQN mode
    
    # Add UI and safety nodes
    ld.add_action(TimerAction(period=12.0, actions=[goal_clicker_node]))
    ld.add_action(TimerAction(period=12.0, actions=[obstacle_monitor_node]))
    ld.add_action(TimerAction(period=12.0, actions=[recorder_node]))
    
    # Add RViz
    ld.add_action(TimerAction(period=15.0, actions=[rviz_node]))
    
    return ld

