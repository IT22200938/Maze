# TurtleBot3 Autonomous Navigation with SLAM, Nav2, and DQN

A comprehensive ROS 2 Jazzy project for simulating and autonomously navigating a TurtleBot3 Burger robot in a maze-like Gazebo Sim environment using SLAM, Nav2, and Deep Q-learning.

## 🎯 Project Overview

This project implements a complete autonomous navigation system with the following features:

- **Simulation Layer**: Custom maze environment in Gazebo Sim with 0.5m corridors
- **Perception Layer**: SLAM using `slam_toolbox` for mapping and localization
- **Navigation Layer**: Nav2 stack with A*/Dijkstra planning algorithms
- **Learning Layer**: Deep Q-Learning (DQN) agent for reinforcement learning-based navigation
- **Safety Layer**: Obstacle monitoring with emergency stop at 0.18m threshold
- **Evaluation Layer**: Performance logging and metrics comparison

## 📋 System Requirements

| Component | Version / Requirement |
|-----------|----------------------|
| Operating System | Ubuntu 24.04 (Noble Numbat) |
| ROS Distribution | ROS 2 Jazzy Jalisco |
| Simulator | Gazebo Sim (version 8.9.0) |
| Robot Model | TurtleBot3 Burger |
| SLAM Framework | slam_toolbox |
| Navigation Stack | Nav2 |
| Languages | Python 3.12+, C++17 |
| Build Tool | colcon |
| Visualization | RViz2 |

## 📁 Project Structure

```
turtlebot3_autonav/
├── world/
│   └── maze.world                      # Custom maze Gazebo world
│
├── urdf/
│   └── turtlebot3_burger.urdf.xacro   # Robot description with LiDAR
│
├── config/
│   ├── slam_params.yaml                # SLAM configuration
│   └── nav2_params.yaml                # Nav2 stack parameters
│
├── launch/
│   ├── full_sim_launch.py              # Main launch file
│   ├── online_slam.launch.py           # SLAM launch file
│   └── nav2_bringup.launch.py          # Nav2 launch file
│
├── scripts/
│   ├── ui/
│   │   └── goal_clicker_node.py        # RViz goal interface
│   ├── dqn_agent/
│   │   ├── dqn_node.py                 # DQN agent ROS2 node
│   │   └── train.py                    # Training script
│   └── tools/
│       └── recorder_node.py            # Performance recorder
│
├── src/
│   └── safety/
│       └── obstacle_monitor.cpp        # C++ safety node
│
├── rviz/
│   └── navigation.rviz                 # RViz configuration
│
├── meshes/                             # Robot meshes (optional)
├── package.xml                         # Package manifest
├── CMakeLists.txt                      # CMake build file
├── setup.py                            # Python setup file
└── README.md                           # This file
```

## 🚀 Installation

### 1. Prerequisites

Ensure you have ROS 2 Jazzy installed:

```bash
# Source ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash
```

### 2. Install Dependencies

```bash
# Install required ROS 2 packages
sudo apt update
sudo apt install -y \
    ros-jazzy-gazebo-ros-pkgs \
    ros-jazzy-slam-toolbox \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-turtlebot3* \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-xacro \
    ros-jazzy-ros-gz \
    python3-pip

# Install Python dependencies
pip3 install torch numpy
```

### 3. Create Workspace and Build

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone or copy the package
cp -r /path/to/turtlebot3_autonav .

# Build the workspace
cd ~/ros2_ws
colcon build --symlink-install

# Source the workspace
source ~/ros2_ws/install/setup.bash
```

## 🎮 Usage

### Basic Launch (Nav2 Mode)

Launch the complete simulation with Nav2 navigation:

```bash
ros2 launch turtlebot3_autonav full_sim_launch.py
```

This will start:
- Gazebo Sim with the maze world
- TurtleBot3 Burger robot
- SLAM Toolbox for mapping
- Nav2 navigation stack
- RViz2 for visualization
- Safety monitoring
- Performance recording

### Setting Navigation Goals

1. **In RViz2**:
   - Wait for the map to appear
   - Click the "2D Goal Pose" button in the toolbar
   - Click on the map to set the destination
   - The robot will navigate autonomously

2. **Programmatically**:
   ```bash
   ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped "{
     header: {frame_id: 'map'},
     pose: {position: {x: 2.5, y: 2.5, z: 0.0}}
   }"
   ```

### Launch Options

#### Use DQN Agent Instead of Nav2

```bash
ros2 launch turtlebot3_autonav full_sim_launch.py nav_mode:=dqn
```

#### Disable SLAM (use pre-made map)

```bash
ros2 launch turtlebot3_autonav full_sim_launch.py use_slam:=false
```

#### Disable RViz

```bash
ros2 launch turtlebot3_autonav full_sim_launch.py use_rviz:=false
```

#### Disable Safety Monitor

```bash
ros2 launch turtlebot3_autonav full_sim_launch.py enable_safety:=false
```

## 🤖 DQN Training

### Training the DQN Agent

1. **Start simulation in training mode**:
   ```bash
   ros2 run turtlebot3_autonav dqn_node.py --ros-args -p training_mode:=true
   ```

2. **Run training script**:
   ```bash
   ros2 run turtlebot3_autonav train.py --episodes 100
   ```

3. **Monitor training**:
   - Check terminal output for episode rewards
   - Model checkpoints are saved periodically

### Using Trained Model

```bash
ros2 run turtlebot3_autonav dqn_node.py --ros-args \
    -p training_mode:=false \
    -p model_path:=/path/to/dqn_model.pth
```

## 📊 Performance Evaluation

### Viewing Logs

Performance logs are saved to `./navigation_logs/`:

```bash
# View latest session
cat navigation_logs/session_nav2_*.json

# View all sessions
cat navigation_logs/all_sessions_nav2.json
```

### Metrics Recorded

- **Duration**: Time to reach goal (seconds)
- **Distance**: Total distance traveled (meters)
- **Path Efficiency**: Ratio of straight-line to actual distance (%)
- **Collisions**: Number of emergency stops
- **Success Rate**: Goal achievement rate
- **Average Speed**: Mean velocity during navigation

### Comparing Nav2 vs DQN

```bash
# Run Nav2 evaluation
ros2 launch turtlebot3_autonav full_sim_launch.py nav_mode:=nav2

# Run DQN evaluation
ros2 launch turtlebot3_autonav full_sim_launch.py nav_mode:=dqn

# Compare results
python3 -c "
import json
nav2 = json.load(open('navigation_logs/all_sessions_nav2.json'))
dqn = json.load(open('navigation_logs/all_sessions_dqn.json'))
print('Nav2 avg duration:', sum(s['duration_seconds'] for s in nav2)/len(nav2))
print('DQN avg duration:', sum(s['duration_seconds'] for s in dqn)/len(dqn))
"
```

## 🛠️ Configuration

### Adjusting Robot Parameters

Edit `urdf/turtlebot3_burger.urdf.xacro`:
- Wheel dimensions
- LiDAR range and resolution
- Robot size and mass

### Tuning Nav2 Parameters

Edit `config/nav2_params.yaml`:
- Planner settings (A*/Dijkstra)
- Controller parameters (DWB)
- Costmap configurations
- Recovery behaviors

### Modifying SLAM Parameters

Edit `config/slam_params.yaml`:
- Loop closure settings
- Scan matching parameters
- Map resolution
- Update rates

### Changing Safety Thresholds

```bash
ros2 run turtlebot3_autonav obstacle_monitor --ros-args \
    -p safety_distance:=0.25
```

## 🏗️ System Architecture

### Layer Diagram

```
┌─────────────────────────────────────────────────────┐
│                  User Interface                      │
│              (RViz2 + Goal Clicker)                  │
└─────────────────────────────────────────────────────┘
                         ↓
┌─────────────────────────────────────────────────────┐
│              Navigation & Planning                   │
│         (Nav2 Stack / DQN Agent)                     │
└─────────────────────────────────────────────────────┘
                         ↓
┌─────────────────────────────────────────────────────┐
│                  Perception                          │
│           (SLAM Toolbox + Sensor Fusion)             │
└─────────────────────────────────────────────────────┘
                         ↓
┌─────────────────────────────────────────────────────┐
│                    Safety                            │
│              (Obstacle Monitor)                      │
└─────────────────────────────────────────────────────┘
                         ↓
┌─────────────────────────────────────────────────────┐
│                  Simulation                          │
│      (Gazebo Sim + TurtleBot3 + Maze World)          │
└─────────────────────────────────────────────────────┘
```

### Topic Flow

| Topic | Type | Description |
|-------|------|-------------|
| `/scan` | LaserScan | LiDAR sensor data |
| `/odom` | Odometry | Robot odometry |
| `/cmd_vel` | Twist | Velocity commands |
| `/map` | OccupancyGrid | SLAM-generated map |
| `/goal_pose` | PoseStamped | Navigation goal |
| `/plan` | Path | Global path plan |
| `/emergency_stop` | Bool | Safety alert |

## 🔧 Troubleshooting

### Gazebo Not Starting

```bash
# Check Gazebo installation
gz sim --version

# Reset Gazebo
killall gz
rm -rf ~/.gz/*
```

### Robot Not Moving

1. Check if velocity commands are published:
   ```bash
   ros2 topic echo /cmd_vel
   ```

2. Verify emergency stop is not active:
   ```bash
   ros2 topic echo /emergency_stop
   ```

### SLAM Not Working

1. Check scan data:
   ```bash
   ros2 topic echo /scan
   ```

2. Verify TF frames:
   ```bash
   ros2 run tf2_tools view_frames
   ```

### Nav2 Not Planning

1. Ensure map is available:
   ```bash
   ros2 topic echo /map --once
   ```

2. Check Nav2 lifecycle nodes:
   ```bash
   ros2 lifecycle list
   ```

### DQN Performance Issues

- Increase training episodes
- Adjust learning rate and epsilon decay
- Collect more diverse training data
- Use GPU if available

## 📝 Development

### Adding New Features

1. **Custom Planner**: Implement in `src/` and add to CMakeLists.txt
2. **New Sensor**: Add to URDF and configure in Gazebo plugin
3. **Additional Safety**: Extend `obstacle_monitor.cpp`
4. **Enhanced DQN**: Modify network architecture in `dqn_node.py`

### Building Individual Components

```bash
# Build only specific package
colcon build --packages-select turtlebot3_autonav

# Build with verbose output
colcon build --event-handlers console_direct+
```

## 🤝 Contributing

Contributions are welcome! Please:

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

## 📄 License

This project is licensed under the Apache License 2.0.

## 👥 Authors

- Your Name (your.email@example.com)

## 🙏 Acknowledgments

- ROS 2 Navigation Team
- SLAM Toolbox contributors
- TurtleBot3 community
- Gazebo Sim developers

## 📚 References

- [ROS 2 Documentation](https://docs.ros.org/en/jazzy/)
- [Nav2 Documentation](https://navigation.ros.org/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [TurtleBot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/)
- [Gazebo Sim](https://gazebosim.org/)

## 📧 Support

For issues and questions:
- Open an issue on GitHub
- Check existing documentation
- Contact maintainers

---

**Note**: This project is designed for ROS 2 Jazzy on Ubuntu 24.04. Compatibility with other versions is not guaranteed.

