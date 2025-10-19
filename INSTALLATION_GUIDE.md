# Complete Installation Guide - Gazebo Harmonic with ROS 2 Jazzy

## Understanding the Issue

The original implementation mixed **Gazebo Classic** packages (`gazebo_ros`) with **Gazebo Harmonic** commands (`gz sim`). This has now been corrected.

## Required Installations

### 1. Base ROS 2 Jazzy (You should already have this)

```bash
# Ubuntu 24.04 LTS
# ROS 2 Jazzy
sudo apt update
source /opt/ros/jazzy/setup.bash
```

### 2. Gazebo Harmonic (You should already have this)

```bash
# Check if installed
gz sim --version

# Should show: Gazebo Sim, version 8.x.x (Harmonic)

# If not installed:
sudo apt install gz-harmonic
```

### 3. ROS-Gazebo Bridge Packages ⚠️ **NEW REQUIREMENT**

These are the key packages needed for ROS 2 Jazzy to work with Gazebo Harmonic:

```bash
# Install ros_gz packages
sudo apt update
sudo apt install ros-jazzy-ros-gz-sim
sudo apt install ros-jazzy-ros-gz-bridge
sudo apt install ros-jazzy-ros-gz-spawn-entity
sudo apt install ros-jazzy-ros-gz-interfaces
sudo apt install ros-jazzy-ros-gz-image

# Verify installation
ros2 pkg list | grep ros_gz
```

### 4. TurtleBot3 Packages

```bash
# If you followed the original setup guide, you should have:
cd ~/turtlebot3_ws/src/
git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

cd ~/turtlebot3_ws
colcon build --symlink-install
```

### 5. Navigation and SLAM Packages

```bash
# Cartographer
sudo apt install ros-jazzy-cartographer
sudo apt install ros-jazzy-cartographer-ros

# Navigation2
sudo apt install ros-jazzy-navigation2
sudo apt install ros-jazzy-nav2-bringup
```

## Build This Package

```bash
cd ~/turtlebot3_ws/src/turtlebot3_maze_navigation
chmod +x build_and_test.sh
./build_and_test.sh
```

## Verification

Run the test script:

```bash
cd ~/turtlebot3_ws
source install/setup.bash
ros2 run turtlebot3_maze_navigation test_setup.py
```

## Launch Options

### Option 1: Main Launch File (Recommended)

Uses `ros_gz_sim` and `ros_gz_bridge` for proper Gazebo Harmonic integration:

```bash
cd ~/turtlebot3_ws
source install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_maze_navigation gazebo_maze.launch.py
```

**Requirements:**
- `ros_gz_sim` packages installed
- `turtlebot3_description` package available

### Option 2: Simple Launch File

Falls back to basic Gazebo launch:

```bash
ros2 launch turtlebot3_maze_navigation gazebo_maze_simple.launch.py
```

**Use this if:**
- You have issues with ros_gz packages
- TurtleBot3 description not found
- Want a simpler setup

## What Each Package Does

### `ros_gz_sim`
- Provides integration between ROS 2 and Gazebo Sim
- Includes spawn utilities
- Command: `ros2 run ros_gz_sim create`

### `ros_gz_bridge`
- Bridges ROS 2 topics to Gazebo topics
- Essential for sensor data (LiDAR, odometry)
- Essential for control (cmd_vel)
- Command: `ros2 run ros_gz_bridge parameter_bridge`

### `ros_gz_interfaces`
- Message definitions for Gazebo-ROS communication

## Topic Bridging Explained

When you launch the simulation, the bridge creates these connections:

```
ROS 2 Side                    Gazebo Harmonic Side
-----------                   --------------------
/scan                    <-->  /world/maze/model/turtlebot3/link/base_scan/sensor/lidar/scan
/cmd_vel                 <-->  /model/turtlebot3/cmd_vel
/odom                    <-->  /model/turtlebot3/odometry
/tf                      <-->  (pose transforms)
```

Your ROS 2 nodes (Nav2, Cartographer) use the left side.
Gazebo uses the right side.
The bridge translates between them.

## Troubleshooting

### Issue 1: "Package 'ros_gz_sim' not found"

**Solution:**
```bash
sudo apt install ros-jazzy-ros-gz-sim
source /opt/ros/jazzy/setup.bash
cd ~/turtlebot3_ws
colcon build --symlink-install
source install/setup.bash
```

### Issue 2: Robot doesn't spawn

**Option A:** Check TurtleBot3 description:
```bash
ros2 pkg list | grep turtlebot3_description
# If not found:
cd ~/turtlebot3_ws/src
git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3.git
cd ~/turtlebot3_ws
colcon build
```

**Option B:** Use simple launch file:
```bash
ros2 launch turtlebot3_maze_navigation gazebo_maze_simple.launch.py
```

### Issue 3: No scan topic

**Check if bridge is running:**
```bash
ros2 node list | grep bridge
ros2 topic list | grep scan
```

**Restart with bridge:**
```bash
# The main launch file includes the bridge
# If manually running:
ros2 run ros_gz_bridge parameter_bridge /scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan
```

### Issue 4: Gazebo Harmonic not installed

**Check version:**
```bash
gz sim --version
```

**If shows Gazebo Classic or not found:**
```bash
sudo apt remove gazebo*  # Remove old Gazebo
sudo apt install gz-harmonic
gz sim --version  # Should show Harmonic
```

## Alternative: Include Robot in World File

If spawning is problematic, edit `worlds/maze_world.sdf` and add:

```xml
<sdf version="1.8">
  <world name="maze_world">
    <!-- ... existing world content ... -->
    
    <!-- Include TurtleBot3 -->
    <model name="turtlebot3_burger">
      <pose>-1.25 1.25 0.01 0 0 0</pose>
      <include>
        <uri>model://turtlebot3_burger</uri>
      </include>
    </model>
    
  </world>
</sdf>
```

Then launch with just:
```bash
gz sim -r ~/turtlebot3_ws/src/turtlebot3_maze_navigation/worlds/maze_world.sdf
```

## Summary

**What was wrong:**
- Used `gazebo_ros` packages (for Gazebo Classic)
- With `gz sim` command (for Gazebo Harmonic)
- These are incompatible

**What's fixed:**
- Use `ros_gz_sim` packages (for Gazebo Harmonic)
- Use `ros_gz_bridge` for topic bridging
- Use `gz sim` command (correct)
- Proper integration

**What you need:**
```bash
# Install these:
sudo apt install ros-jazzy-ros-gz-sim
sudo apt install ros-jazzy-ros-gz-bridge
sudo apt install ros-jazzy-ros-gz-spawn-entity

# Build package:
cd ~/turtlebot3_ws
colcon build --packages-select turtlebot3_maze_navigation
source install/setup.bash

# Launch:
ros2 launch turtlebot3_maze_navigation gazebo_maze.launch.py
```

## Quick Setup Script

Save this as `setup_ros_gz.sh`:

```bash
#!/bin/bash
echo "Installing ROS-Gazebo bridge packages..."
sudo apt update
sudo apt install -y \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-spawn-entity \
    ros-jazzy-ros-gz-interfaces \
    ros-jazzy-ros-gz-image

echo "Done! Now build your workspace:"
echo "cd ~/turtlebot3_ws && colcon build && source install/setup.bash"
```

Run with: `chmod +x setup_ros_gz.sh && ./setup_ros_gz.sh`

## References

- [Gazebo Harmonic Docs](https://gazebosim.org/docs/harmonic)
- [ros_gz Repository](https://github.com/gazebosim/ros_gz)
- [ROS 2 + Gazebo Guide](https://docs.ros.org/en/jazzy/How-To-Guides/Using-Gazebo-Simulator.html)

Thank you for catching the Gazebo Classic vs Harmonic issue! The project is now properly configured.

