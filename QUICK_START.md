# Quick Start Guide - TurtleBot3 Maze Navigation

This is a condensed guide to get you up and running quickly.

## Prerequisites Check

```bash
# Check ROS 2 installation
ros2 --version  # Should show: ros2 doctor version jazzy

# Check Gazebo installation
gz sim --version  # Should show Gazebo Harmonic

# Check TurtleBot3 installation
ls ~/turtlebot3_ws/src/turtlebot3

# Set environment
export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=30
```

## Build the Package

```bash
cd ~/turtlebot3_ws
colcon build --packages-select turtlebot3_maze_navigation
source install/setup.bash
```

## Quick Test (5 minutes)

### 1. Launch Maze (Terminal 1)
```bash
cd ~/turtlebot3_ws
source install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_maze_navigation gazebo_maze.launch.py
```

**Expected:** Gazebo opens with 6x6 maze and robot at top-left corner.

### 2. Test Manual Control (Terminal 2)
```bash
cd ~/turtlebot3_ws
source install/setup.bash
ros2 run turtlebot3_teleop teleop_keyboard
```

**Test:** Press 'w' to move forward, 's' to stop. Robot should respond.

## Full Workflow

### Phase 1: Mapping (15-20 minutes)

**Terminal 1 - Gazebo:**
```bash
cd ~/turtlebot3_ws && source install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_maze_navigation gazebo_maze.launch.py
```

**Terminal 2 - SLAM:**
```bash
cd ~/turtlebot3_ws && source install/setup.bash
ros2 launch turtlebot3_maze_navigation cartographer.launch.py
```

**Terminal 3 - Control:**
```bash
cd ~/turtlebot3_ws && source install/setup.bash
ros2 run turtlebot3_teleop teleop_keyboard
```

**Action:** Drive robot around entire maze slowly for 10 minutes.

**Terminal 4 - Save Map:**
```bash
cd ~/turtlebot3_ws/src/turtlebot3_maze_navigation/maps
ros2 run nav2_map_server map_saver_cli -f maze_map
```

**Result:** `maze_map.yaml` and `maze_map.pgm` created.

### Phase 2: Navigation (10 minutes)

**Close all terminals from Phase 1.**

**Terminal 1 - Gazebo:**
```bash
cd ~/turtlebot3_ws && source install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_maze_navigation gazebo_maze.launch.py
```

**Terminal 2 - Navigation:**
```bash
cd ~/turtlebot3_ws && source install/setup.bash
ros2 launch turtlebot3_maze_navigation navigation.launch.py
```

**In RViz:**
1. Click "2D Pose Estimate"
2. Click on robot location and drag to set orientation
3. Click "2D Goal Pose" (or Nav2 Goal)
4. Click destination and drag to set orientation
5. Watch robot navigate autonomously!

**Test 3 goals:**
- Near: Adjacent corridor
- Medium: Center of maze
- Far: Opposite corner

### Phase 3: Deep Q-Learning (Optional, for advanced users)

**Terminal 1 - Gazebo:**
```bash
cd ~/turtlebot3_ws && source install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_maze_navigation gazebo_maze.launch.py
```

**Terminal 2 - DQL:**
```bash
cd ~/turtlebot3_ws && source install/setup.bash
ros2 run turtlebot3_maze_navigation dql_navigator.py
```

**Note:** This requires training (several hours). For demo, you can load pre-trained model.

## Common Issues & Fixes

### Gazebo doesn't start
```bash
killall gz
gz sim --version  # Verify installation
```

### "Package not found"
```bash
cd ~/turtlebot3_ws
colcon build --packages-select turtlebot3_maze_navigation
source install/setup.bash
```

### Robot doesn't move
```bash
# Check if cmd_vel topic exists
ros2 topic list | grep cmd_vel

# Check if messages are being published
ros2 topic echo /cmd_vel
```

### Map not loading
```bash
# Verify map files exist
ls ~/turtlebot3_ws/src/turtlebot3_maze_navigation/maps/

# Check map topic
ros2 topic echo /map --once
```

### Navigation fails
- Ensure initial pose is set correctly
- Check if goal is in free space (not in wall)
- Clear costmaps:
```bash
ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntirelyGlobalCostmap
```

## Recording Performance Data

### Time to goal:
```bash
# Start timer when setting goal
# Stop when robot reaches goal (within 0.1m)
```

### Path length:
```bash
ros2 topic echo /plan
# Sum distances between consecutive waypoints
```

### Success rate:
- Count successful arrivals / total attempts

## For VIVA Demo

**Preparation:**
1. Test everything the day before
2. Have 3 goal locations ready
3. Practice explaining your approach
4. Keep backup terminals with commands ready

**During Demo:**
1. Explain maze design
2. Show SLAM mapping (brief)
3. Demonstrate Nav2 navigation (3 goals)
4. Explain algorithms (A*, costmaps)
5. Show performance comparison table

**Be ready to answer:**
- How does SLAM work?
- What is A* algorithm?
- How do costmaps work?
- Why did you choose these parameters?
- What challenges did you face?

## Next Steps

1. ✅ Build and test basic simulation
2. ✅ Complete SLAM mapping
3. ✅ Test Nav2 navigation
4. ⬜ Record performance metrics
5. ⬜ Train DQL (optional)
6. ⬜ Write report
7. ⬜ Prepare demo

## Support

- Check main `README.md` for detailed explanations
- Review code comments in launch files
- Test each phase independently
- Use `ros2 topic list` and `ros2 node list` to debug

---

**You've got this! 🚀**

