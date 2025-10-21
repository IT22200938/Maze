# Quick Start Guide

Get up and running with TurtleBot3 autonomous navigation in 5 minutes!

## Prerequisites

✅ Ubuntu 24.04  
✅ ROS 2 Jazzy installed  
✅ Gazebo Sim installed  

## Installation (One-time Setup)

```bash
# 1. Install dependencies
sudo apt update
sudo apt install -y \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-interfaces \
    ros-jazzy-slam-toolbox \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-xacro \
    python3-pip

pip3 install torch numpy

# 2. Create workspace and build
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src
cp -r /path/to/turtlebot3_autonav .

cd ~/turtlebot3_ws
colcon build --symlink-install
source install/setup.bash
```

## Running the Simulation

### Option 1: Full System (Recommended)

```bash
# Terminal 1: Launch everything
source ~/turtlebot3_ws/install/setup.bash
ros2 launch turtlebot3_autonav full_sim_launch.py
```

Wait for RViz to open (15-20 seconds), then:
1. Use "2D Goal Pose" tool in RViz
2. Click on the map to set a destination
3. Watch the robot navigate!

### Option 2: Step-by-Step

```bash
# Terminal 1: Gazebo + Robot
ros2 launch turtlebot3_autonav gazebo_only.launch.py

# Terminal 2: SLAM
ros2 launch turtlebot3_autonav online_slam.launch.py

# Terminal 3: Navigation
ros2 launch turtlebot3_autonav nav2_bringup.launch.py

# Terminal 4: RViz
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix turtlebot3_autonav)/share/turtlebot3_autonav/rviz/navigation.rviz
```

## Common Tasks

### Set Goal via Command Line

```bash
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped "
{
  header: {frame_id: 'map'},
  pose: {
    position: {x: 2.0, y: 2.0, z: 0.0},
    orientation: {w: 1.0}
  }
}"
```

### View Performance Logs

```bash
cat navigation_logs/all_sessions_nav2.json | python3 -m json.tool
```

### Switch to DQN Mode

```bash
ros2 launch turtlebot3_autonav full_sim_launch.py nav_mode:=dqn
```

## Troubleshooting

**Robot not moving?**
- Check: `ros2 topic echo /cmd_vel`
- Solution: Make sure navigation started successfully

**Map not showing in RViz?**
- Wait 5-10 seconds for SLAM to initialize
- Check: `ros2 topic echo /map --once`

**Gazebo crashes?**
- Restart: `killall gz && ros2 launch turtlebot3_autonav full_sim_launch.py`

## Next Steps

- 📖 Read the full [README.md](README.md)
- 🔧 Customize [config/nav2_params.yaml](config/nav2_params.yaml)
- 🤖 Train DQN agent with [scripts/dqn_agent/train.py](scripts/dqn_agent/train.py)
- 📊 Analyze performance with recorder logs

## Video Demo

*TODO: Add link to demo video*

## Support

- Issues? Open a GitHub issue
- Questions? Check the README.md
- Contributions? See CONTRIBUTING.md

Happy navigating! 🚀

