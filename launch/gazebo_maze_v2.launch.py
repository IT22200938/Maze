#!/usr/bin/env python3

"""
Simplified Gazebo Harmonic Launch for TurtleBot3 Maze
This version is more reliable and easier to debug
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directory
    pkg_turtlebot3_maze = get_package_share_directory('turtlebot3_maze_navigation')
    world_file = os.path.join(pkg_turtlebot3_maze, 'worlds', 'maze_world.sdf')
    
    # Set TurtleBot3 model
    os.environ['TURTLEBOT3_MODEL'] = 'burger'
    
    # Launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # 1. Launch Gazebo Harmonic with maze world
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen'
    )
    
    # 2. Load robot description
    robot_description = """<?xml version="1.0"?>
<robot name="turtlebot3_burger">
  <link name="base_footprint"/>
  
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.140 0.140 0.143"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.140 0.140 0.143"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0"
               iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>
  
  <joint name="base_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0 0 0.010" rpy="0 0 0"/>
  </joint>
</robot>"""
    
    # 3. Robot State Publisher
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
    
    # 4. ROS-Gazebo Bridge for topics
    # This is CRITICAL for communication between ROS 2 and Gazebo Harmonic
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        remappings=[
            ('/world/default/model/turtlebot3/link/base_scan/sensor/lidar/scan', '/scan'),
            ('/model/turtlebot3/cmd_vel', '/cmd_vel'),
            ('/model/turtlebot3/odometry', '/odom'),
        ],
        output='screen'
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        
        # Launch Gazebo first
        gazebo,
        
        # Start robot state publisher
        robot_state_publisher,
        
        # Start bridge after a short delay to ensure Gazebo is up
        TimerAction(
            period=3.0,
            actions=[bridge]
        )
    ])

