# Project Verification Checklist

Use this checklist to verify that all components are properly installed and configured.

## ✅ Pre-Installation Verification

- [ ] Ubuntu 24.04 installed
- [ ] ROS 2 Jazzy installed
- [ ] Gazebo Sim installed (version 8.9.0+)
- [ ] Python 3.12+ available
- [ ] Git installed (for version control)

**Check Commands:**
```bash
lsb_release -a          # Should show Ubuntu 24.04
ros2 --version          # Should show ROS 2 Jazzy
gz sim --version        # Should show Gazebo 8.9.0+
python3 --version       # Should show 3.12+
```

## ✅ File Structure Verification

Run from project root:

```bash
cd ~/turtlebot3_ws/src/turtlebot3_autonav

# Check critical files exist
ls -1 README.md \
      package.xml \
      CMakeLists.txt \
      setup.py \
      world/maze.world \
      urdf/turtlebot3_burger.urdf.xacro \
      config/slam_params.yaml \
      config/nav2_params.yaml \
      launch/full_sim_launch.py \
      src/safety/obstacle_monitor.cpp

# Should show all files without errors
```

## ✅ Build Verification

```bash
cd ~/turtlebot3_ws

# Clean build
rm -rf build/ install/ log/

# Build package
colcon build --packages-select turtlebot3_autonav

# Expected: BUILD SUCCESSFUL message
# Check for: 0 packages had stderr output

# Source workspace
source install/setup.bash
```

**Success Indicators:**
- [ ] No build errors
- [ ] No missing dependencies
- [ ] All executables built
- [ ] Package found: `ros2 pkg list | grep turtlebot3_autonav`

## ✅ Node Verification

Check if all nodes are available:

```bash
# List executables
ros2 pkg executables turtlebot3_autonav

# Expected output:
# turtlebot3_autonav goal_clicker_node.py
# turtlebot3_autonav dqn_node.py
# turtlebot3_autonav recorder_node.py
# turtlebot3_autonav obstacle_monitor
```

- [ ] goal_clicker_node.py listed
- [ ] dqn_node.py listed
- [ ] recorder_node.py listed
- [ ] obstacle_monitor listed

## ✅ Launch File Verification

```bash
# List launch files
ros2 launch turtlebot3_autonav --show-launch-files

# Check specific launch file
ros2 launch turtlebot3_autonav full_sim_launch.py --show-args

# Expected: List of launch arguments
```

- [ ] full_sim_launch.py available
- [ ] online_slam.launch.py available
- [ ] nav2_bringup.launch.py available
- [ ] gazebo_only.launch.py available

## ✅ Dependency Verification

```bash
# Check ROS 2 dependencies
ros2 pkg xml turtlebot3_autonav

# Check Python dependencies
python3 -c "import torch; import numpy; print('OK')"

# Expected: "OK"
```

**Required ROS 2 Packages:**
- [ ] ros_gz_sim
- [ ] ros_gz_bridge
- [ ] ros_gz_interfaces
- [ ] slam_toolbox
- [ ] nav2_bringup
- [ ] robot_state_publisher
- [ ] xacro

**Required Python Packages:**
- [ ] torch
- [ ] numpy
- [ ] rclpy

## ✅ Gazebo Test

```bash
# Test Gazebo with world file
gz sim $(ros2 pkg prefix turtlebot3_autonav)/share/turtlebot3_autonav/world/maze.world

# Expected: Gazebo opens with maze environment
# Press Ctrl+C to close
```

- [ ] Gazebo opens without errors
- [ ] Maze world loads
- [ ] Walls are visible
- [ ] Ground plane present

## ✅ Robot Model Test

```bash
# Test URDF parsing
xacro $(ros2 pkg prefix turtlebot3_autonav)/share/turtlebot3_autonav/urdf/turtlebot3_burger.urdf.xacro

# Expected: Valid URDF XML output
# Should not show errors
```

- [ ] URDF parses without errors
- [ ] All links defined
- [ ] All joints defined
- [ ] LiDAR sensor included

## ✅ Quick Launch Test

```bash
# Launch Gazebo only
ros2 launch turtlebot3_autonav gazebo_only.launch.py

# Expected within 10 seconds:
# - Gazebo window opens
# - Robot spawns at (0.5, 0.5)
# - No error messages
```

- [ ] Gazebo starts successfully
- [ ] Robot visible in simulation
- [ ] No critical errors in terminal

## ✅ Full System Test

```bash
# Launch complete system
ros2 launch turtlebot3_autonav full_sim_launch.py

# Expected within 30 seconds:
# - Gazebo opens with robot
# - RViz opens with visualization
# - Map begins building
# - No critical errors
```

**Check in separate terminals while running:**

```bash
# Terminal 2: Check topics
ros2 topic list

# Expected topics:
# /scan
# /odom
# /map
# /cmd_vel
# /goal_pose
# /plan
```

```bash
# Terminal 3: Check nodes
ros2 node list

# Expected nodes:
# /slam_toolbox
# /goal_clicker
# /obstacle_monitor
# /performance_recorder
# Plus Nav2 nodes...
```

```bash
# Terminal 4: Check transforms
ros2 run tf2_ros tf2_echo map base_footprint

# Expected: Transform values updating
```

**In RViz:**
- [ ] Map display showing
- [ ] Robot model visible
- [ ] LiDAR scan visible
- [ ] Can set 2D goal pose
- [ ] Robot navigates to goal

## ✅ Navigation Test

1. **Set a goal:**
   - Click "2D Goal Pose" in RViz
   - Click on map at position (2.0, 2.0)

2. **Observe:**
   - [ ] Green path appears (global plan)
   - [ ] Purple path appears (local plan)
   - [ ] Robot starts moving
   - [ ] Robot avoids walls
   - [ ] Robot reaches goal

3. **Check logs:**
   ```bash
   ls navigation_logs/
   # Should show session JSON files
   ```

## ✅ Safety System Test

1. **While robot is moving:**
   - Place virtual obstacle in path (or drive robot toward wall)

2. **Observe:**
   - [ ] Robot stops before collision (<0.18m)
   - [ ] Emergency stop message in terminal
   - [ ] Robot resumes after obstacle cleared

## ✅ DQN Mode Test (Optional)

```bash
# Launch with DQN
ros2 launch turtlebot3_autonav full_sim_launch.py nav_mode:=dqn

# Check DQN node
ros2 node info /dqn_agent
```

- [ ] DQN node running
- [ ] Subscribes to /scan, /odom, /goal_pose
- [ ] Publishes to /cmd_vel
- [ ] Model loads (if path provided)

## ✅ Performance Recording Test

```bash
# Check recorder is running
ros2 node info /performance_recorder

# Send test goal
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "..."

# Wait for goal completion, then check logs
cat navigation_logs/all_sessions_nav2.json | python3 -m json.tool
```

- [ ] Recorder node running
- [ ] Logs directory created
- [ ] Session files created
- [ ] Metrics recorded (duration, distance, etc.)

## ✅ Common Issues & Solutions

### Issue: Package not found after build

**Solution:**
```bash
source ~/turtlebot3_ws/install/setup.bash
```

### Issue: Gazebo crashes immediately

**Solution:**
```bash
killall gz
rm -rf ~/.gz/sim
gz sim --version
```

### Issue: No map appearing in RViz

**Solution:**
- Wait 10 seconds for SLAM initialization
- Check: `ros2 topic hz /scan` (should show ~10 Hz)
- Restart SLAM: Kill and relaunch

### Issue: Robot not moving despite goal set

**Solution:**
```bash
# Check velocity commands
ros2 topic echo /cmd_vel

# Check emergency stop
ros2 topic echo /emergency_stop

# Check Nav2 status
ros2 lifecycle list
```

### Issue: Build errors for obstacle_monitor

**Solution:**
```bash
# Install missing dependencies
sudo apt install ros-jazzy-rclcpp ros-jazzy-sensor-msgs
cd ~/turtlebot3_ws
colcon build --packages-select turtlebot3_autonav
```

## ✅ Performance Benchmarks

Expected performance metrics (on recommended hardware):

| Metric | Nav2 | DQN (trained) |
|--------|------|---------------|
| Success Rate | 85-95% | 60-80% |
| Avg Time to Goal (3m) | 30-60s | 40-80s |
| Path Efficiency | 70-85% | 50-70% |
| Collision Rate | <5% | 5-15% |

## Final Verification Summary

Total checks: ~50  
Required to pass: ~40 (80%)

If all critical checks pass, the system is ready for use!

## Next Steps After Verification

1. ✅ Read [QUICKSTART.md](QUICKSTART.md) for usage
2. ✅ Explore [ARCHITECTURE.md](ARCHITECTURE.md) for system details
3. ✅ Customize parameters in `config/` files
4. ✅ Run automated tests with `scripts/test_navigation.sh`
5. ✅ Compare Nav2 vs DQN with `scripts/compare_performance.py`

## Support

If verification fails:
1. Check error messages carefully
2. Review [INSTALL.md](INSTALL.md)
3. Ensure all dependencies installed
4. Check ROS 2 and Gazebo versions
5. Consult [README.md](README.md) troubleshooting section

---

**Last Updated:** 2024  
**Version:** 1.0.0  
**Compatible with:** ROS 2 Jazzy, Ubuntu 24.04

