#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

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
    
    # Get robot description (convert URDF to SDF for Gazebo Harmonic)
    # TurtleBot3 URDF location (you may need to adjust path based on your turtlebot3 installation)
    urdf_file = PathJoinSubstitution([
        FindPackageShare('turtlebot3_description'),
        'urdf',
        'turtlebot3_burger.urdf'
    ])
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': open(urdf_file.perform(None)).read()
        }]
    )
    
    # Spawn TurtleBot3 using ros_gz_spawn_entity
    spawn_turtlebot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'turtlebot3_burger',
            '-file', urdf_file,
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
            '-allow_renaming', 'true'
        ],
        output='screen'
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
        gz_sim,
        robot_state_publisher,
        spawn_turtlebot,
        bridge
    ])

