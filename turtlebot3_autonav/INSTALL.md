# Installation Guide

## Quick Start

### 1. System Requirements

- **OS**: Ubuntu 24.04 (Noble Numbat)
- **ROS**: ROS 2 Jazzy Jalisco
- **Gazebo**: Gazebo Sim 8.9.0 or later
- **Python**: 3.12 or later
- **CMake**: 3.22 or later

### 2. Install ROS 2 Jazzy

```bash
# Add ROS 2 repository
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -
sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Install ROS 2 Jazzy
sudo apt update
sudo apt install ros-jazzy-desktop

# Source ROS 2
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 3. Install Gazebo Sim

```bash
# Install Gazebo
sudo apt install gz-harmonic

# Verify installation
gz sim --version
```

### 4. Install Dependencies

```bash
# Install ROS 2 packages
sudo apt install -y \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-interfaces \
    ros-jazzy-slam-toolbox \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-xacro \
    python3-colcon-common-extensions \
    python3-pip

# Install Python dependencies
pip3 install -r requirements.txt
```

### 5. Build the Package

```bash
# Create workspace
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src

# Copy or clone the package
# (Assuming package is in ~/Downloads/turtlebot3_autonav)
cp -r ~/Downloads/turtlebot3_autonav .

# Navigate to workspace root
cd ~/turtlebot3_ws

# Build
colcon build --symlink-install

# Source the workspace
source install/setup.bash

# Add to bashrc for convenience
echo "source ~/turtlebot3_ws/install/setup.bash" >> ~/.bashrc
```

### 6. Verify Installation

```bash
# Check if package is found
ros2 pkg list | grep turtlebot3_autonav

# List available launch files
ros2 launch turtlebot3_autonav --show-args full_sim_launch.py
```

## Troubleshooting

### Issue: Gazebo command not found

**Solution**:
```bash
# Ensure Gazebo is installed
sudo apt install gz-harmonic

# Add to PATH if needed
export GZ_VERSION=harmonic
```

### Issue: Cannot find package 'turtlebot3_autonav'

**Solution**:
```bash
# Make sure workspace is sourced
source ~/turtlebot3_ws/install/setup.bash

# Rebuild if necessary
cd ~/turtlebot3_ws
colcon build --symlink-install --packages-select turtlebot3_autonav
```

### Issue: Python module 'torch' not found

**Solution**:
```bash
# Install PyTorch
pip3 install torch
```

### Issue: xacro command not found

**Solution**:
```bash
# Install xacro
sudo apt install ros-jazzy-xacro
```

## Next Steps

After successful installation:

1. **Test Gazebo Only**:
   ```bash
   ros2 launch turtlebot3_autonav gazebo_only.launch.py
   ```

2. **Run Full Simulation**:
   ```bash
   ros2 launch turtlebot3_autonav full_sim_launch.py
   ```

3. **Check the README** for usage instructions

## Getting Help

If you encounter issues:

1. Check the main README.md
2. Verify all dependencies are installed
3. Ensure you're using Ubuntu 24.04 and ROS 2 Jazzy
4. Check ROS 2 and Gazebo logs for errors

## Additional Resources

- [ROS 2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [Gazebo Sim Documentation](https://gazebosim.org/)
- [Nav2 Documentation](https://navigation.ros.org/)

