#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directories
    pkg_turtlebot3_maze = get_package_share_directory('turtlebot3_maze_navigation')
    
    # Paths
    world_file = os.path.join(pkg_turtlebot3_maze, 'worlds', 'maze_world.sdf')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='-1.25')
    y_pose = LaunchConfiguration('y_pose', default='1.25')
    z_pose = LaunchConfiguration('z_pose', default='0.01')
    
    # Set TurtleBot3 model
    os.environ['TURTLEBOT3_MODEL'] = 'burger'
    
    # Launch Gazebo Harmonic with the maze world
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen'
    )
    
    # Get robot description
    # Try to find TurtleBot3 URDF
    try:
        pkg_turtlebot3_description = get_package_share_directory('turtlebot3_description')
        urdf_file_path = os.path.join(pkg_turtlebot3_description, 'urdf', 'turtlebot3_burger.urdf')
        
        with open(urdf_file_path, 'r') as urdf_file:
            robot_description = urdf_file.read()
    except:
        # Fallback: Create a simple robot description
        robot_description = """<?xml version="1.0"?>
<robot name="turtlebot3_burger">
  <link name="base_footprint"/>
  <link name="base_link"/>
  <joint name="base_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0 0 0.010" rpy="0 0 0"/>
  </joint>
</robot>"""
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }]
    )
    
    # Spawn TurtleBot3 using ros_gz_sim
    # Note: For now, we'll spawn using the Gazebo service after Gazebo starts
    # This is a simplified approach that works better
    spawn_turtlebot = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-name', 'turtlebot3_burger',
            '-string', robot_description,
            '-x', str(-1.25),
            '-y', str(1.25),
            '-z', str(0.01)
        ],
        output='screen',
        shell=False
    )
    
    # ROS-Gazebo Bridge for topics
    # This bridges Gazebo topics to ROS 2 topics
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
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
        
        # Launch Gazebo first
        gz_sim,
        
        # Start robot state publisher
        robot_state_publisher,
        
        # Wait for Gazebo to start, then spawn robot and start bridge
        TimerAction(
            period=3.0,
            actions=[spawn_turtlebot, bridge]
        )
    ])

