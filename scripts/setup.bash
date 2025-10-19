#!/bin/bash

# Setup script for TurtleBot3 Maze Navigation
# Run this script to set up your environment

echo "=========================================="
echo "TurtleBot3 Maze Navigation Setup"
echo "=========================================="

# Source ROS 2
echo "Sourcing ROS 2 Jazzy..."
source /opt/ros/jazzy/setup.bash

# Source workspace
echo "Sourcing workspace..."
source ~/turtlebot3_ws/install/setup.bash

# Set environment variables
echo "Setting environment variables..."
export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=30
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_ws/src/turtlebot3_maze_navigation

echo ""
echo "Environment setup complete!"
echo ""
echo "Environment Variables:"
echo "  TURTLEBOT3_MODEL=$TURTLEBOT3_MODEL"
echo "  ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo ""
echo "To use these settings in this terminal, run:"
echo "  source scripts/setup.bash"
echo ""
echo "To make permanent, add to ~/.bashrc:"
echo "  echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc"
echo "  echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc"
echo "  echo 'export ROS_DOMAIN_ID=30' >> ~/.bashrc"
echo ""
echo "Ready to launch! Try:"
echo "  ros2 launch turtlebot3_maze_navigation gazebo_maze.launch.py"
echo "=========================================="

