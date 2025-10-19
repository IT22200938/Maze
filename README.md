# TurtleBot3 Maze Navigation Project

This project implements autonomous navigation for a TurtleBot3 Burger robot in a 6x6 maze environment using ROS 2 Jazzy and Gazebo.

## Project Overview

- **Maze Size**: 6x6 grid (3m x 3m total, 0.5m per cell)
- **Robot**: TurtleBot3 Burger with LiDAR
- **ROS Version**: ROS 2 Jazzy Jalisco
- **Simulator**: Gazebo Harmonic
- **Navigation**: Nav2 stack (A* algorithm) + Deep Q-Learning

## Prerequisites

Ensure you have completed the setup from the Quick Start Guide:
- Ubuntu 24.04 LTS
- ROS 2 Jazzy installed
- Gazebo Harmonic installed
- TurtleBot3 packages installed
- Cartographer and Nav2 packages installed

## Project Structure

```
turtlebot3_maze_navigation/
├── launch/
│   ├── gazebo_maze.launch.py      # Launch Gazebo with maze world
│   ├── cartographer.launch.py     # Launch SLAM mapping
│   ├── navigation.launch.py       # Launch Nav2 navigation
│   └── teleop.launch.py           # Manual control
├── worlds/
│   └── maze_world.sdf             # 6x6 maze world file
├── config/
│   ├── cartographer.lua           # Cartographer configuration
│   ├── nav2_params.yaml           # Nav2 parameters
│   ├── mapping.rviz               # RViz config for mapping
│   └── navigation.rviz            # RViz config for navigation
├── maps/
│   └── (generated map files)      # Maps created by SLAM
├── scripts/
│   └── dql_navigator.py           # Deep Q-Learning implementation
├── CMakeLists.txt
├── package.xml
└── README.md
```

## Installation

1. **Clone this repository into your workspace:**
   ```bash
   cd ~/turtlebot3_ws/src/
   # Assuming your project is already here
   ```

2. **Build the package:**
   ```bash
   cd ~/turtlebot3_ws
   colcon build --packages-select turtlebot3_maze_navigation
   source install/setup.bash
   ```

3. **Set environment variables (add to ~/.bashrc):**
   ```bash
   echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
   echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_ws/src/turtlebot3_maze_navigation' >> ~/.bashrc
   source ~/.bashrc
   ```

## Step-by-Step Usage Guide

### Step 1: Launch Gazebo with Maze World

Open a terminal and launch the maze simulation:

```bash
cd ~/turtlebot3_ws
source install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_maze_navigation gazebo_maze.launch.py
```

**What to expect:**
- Gazebo window opens with the 6x6 maze
- TurtleBot3 Burger spawns at position (-1.25, 1.25) - top-left corner
- Robot is stationary, waiting for commands

**Verification:**
- Check if robot is visible in Gazebo
- Verify walls are properly placed
- Robot should be on the ground, not floating

### Step 2: Test Teleoperation (Optional but Recommended)

In a new terminal, test manual control:

```bash
cd ~/turtlebot3_ws
source install/setup.bash
ros2 run turtlebot3_teleop teleop_keyboard
```

**Controls:**
- `w` - Move forward
- `x` - Move backward
- `a` - Turn left
- `d` - Turn right
- `s` - Stop
- `SPACE` - Emergency stop

**Test:**
- Drive robot around to ensure it responds to commands
- Verify LiDAR is detecting walls (check `/scan` topic)
- Test collision detection by driving near walls

### Step 3: Create Map using SLAM (Cartographer)

Stop teleoperation (Ctrl+C) and launch SLAM:

```bash
cd ~/turtlebot3_ws
source install/setup.bash
ros2 launch turtlebot3_maze_navigation cartographer.launch.py
```

**What to expect:**
- RViz window opens showing robot and map
- Initially, map is empty
- Green dots show LiDAR scan data

**Mapping the maze:**

In another terminal, run teleoperation:
```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

**Mapping strategy:**
1. Drive slowly through the maze
2. Cover all areas systematically (follow walls)
3. Try to visit every corridor and corner
4. The map will build in real-time in RViz
5. Drive for 5-10 minutes to create complete map

**Save the map:**

When mapping is complete, open a new terminal:
```bash
cd ~/turtlebot3_ws/src/turtlebot3_maze_navigation/maps
ros2 run nav2_map_server map_saver_cli -f maze_map
```

**Result:**
- `maze_map.yaml` - Map configuration
- `maze_map.pgm` - Map image

**Verification:**
- Open `maze_map.pgm` to visually inspect the map
- Ensure all walls are visible
- Map should show clear passages

### Step 4: Autonomous Navigation with Nav2

**Close all previous terminals (Gazebo, SLAM, teleoperation).**

**Restart simulation:**

Terminal 1 - Launch Gazebo:
```bash
cd ~/turtlebot3_ws
source install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_maze_navigation gazebo_maze.launch.py
```

Terminal 2 - Launch Navigation:
```bash
cd ~/turtlebot3_ws
source install/setup.bash
ros2 launch turtlebot3_maze_navigation navigation.launch.py
```

**What to expect:**
- Gazebo with robot
- RViz with map, robot, and costmaps
- Navigation 2 panel visible

**Set initial pose:**
1. In RViz, click "2D Pose Estimate" button
2. Click on robot's approximate location on map
3. Drag to set orientation
4. Green arrow shows robot's estimated pose
5. Wait for AMCL to converge (robot aligns with map)

**Send navigation goal:**
1. Click "2D Goal Pose" button (or use "Nav2 Goal" from Navigation 2 panel)
2. Click destination on the map
3. Drag to set desired orientation
4. Robot plans path (green line) and navigates autonomously

**What to observe:**
- **Global Costmap** (red/blue overlay): Static obstacles from map
- **Local Costmap** (smaller area around robot): Dynamic obstacle detection
- **Green path**: Planned route using A* algorithm
- Robot follows path, avoiding obstacles

**Performance metrics to record:**
- Start time: When goal is set
- End time: When robot reaches goal
- Path length: Check `/plan` topic
- Success rate: Did robot reach goal?

**Test at least 3 different goals:**
1. Short distance (e.g., next corridor)
2. Medium distance (e.g., across maze)
3. Long distance (e.g., opposite corner)

### Step 5: Deep Q-Learning Navigation (Advanced)

This implements reinforcement learning for navigation.

**Launch DQL training/testing:**

```bash
cd ~/turtlebot3_ws
source install/setup.bash
ros2 run turtlebot3_maze_navigation dql_navigator.py
```

**Note:** DQL implementation requires training phase. See `scripts/dql_navigator.py` for details.

## Comparing Navigation Methods

### Method 1: Nav2 (A* Algorithm)
**Advantages:**
- Deterministic path planning
- Fast computation
- Uses global map knowledge
- Proven reliability

**Disadvantages:**
- Requires pre-built map
- Limited dynamic adaptation
- Fixed cost functions

### Method 2: Deep Q-Learning
**Advantages:**
- Learns optimal policy through experience
- Adapts to environment
- Can handle dynamic obstacles
- Potentially finds novel solutions

**Disadvantages:**
- Requires extensive training
- Computationally intensive
- Less predictable
- May not converge to optimal solution

## Recording Results

For your report, record these metrics:

### Performance Metrics

| Method | Goal Location | Time (s) | Path Length (m) | Success | Collisions |
|--------|--------------|----------|-----------------|---------|------------|
| Nav2   | Goal 1       |          |                 |         |            |
| Nav2   | Goal 2       |          |                 |         |            |
| Nav2   | Goal 3       |          |                 |         |            |
| DQL    | Goal 1       |          |                 |         |            |
| DQL    | Goal 2       |          |                 |         |            |
| DQL    | Goal 3       |          |                 |         |            |

### How to measure:

**Time:** 
```bash
# Monitor navigation status
ros2 topic echo /navigate_to_pose/_action/status
```

**Path Length:**
```bash
# Get planned path
ros2 topic echo /plan
# Calculate sum of distances between waypoints
```

**Success:**
- Did robot reach within 0.1m of goal?
- Check goal tolerance in nav2_params.yaml

## Troubleshooting

### Issue: Gazebo doesn't start
- **Solution:** Check if Gazebo is properly installed: `gz sim --version`
- Ensure no other Gazebo instances running: `killall gz`

### Issue: Robot falls through floor
- **Solution:** Check Z position in launch file (should be 0.01)
- Verify physics plugin in world file

### Issue: SLAM map is noisy
- **Solution:** 
  - Drive slower during mapping
  - Adjust Cartographer parameters
  - Ensure good LiDAR data: `ros2 topic echo /scan`

### Issue: Robot won't navigate
- **Solution:**
  - Check if map is loaded: `ros2 topic echo /map`
  - Verify initial pose is set correctly
  - Check costmaps are visible in RViz
  - Ensure goal is in free space (not in wall)

### Issue: Robot gets stuck
- **Solution:**
  - Increase inflation radius in nav2_params.yaml
  - Adjust DWB planner parameters
  - Check local costmap size

### Issue: Path planning fails
- **Solution:**
  - Verify map quality
  - Check if goal is reachable
  - Increase planner tolerance
  - Clear costmaps: `ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntirelyGlobalCostmap`

## Useful Commands

### Check Topics
```bash
ros2 topic list                    # List all topics
ros2 topic echo /scan              # View LiDAR data
ros2 topic echo /odom              # View odometry
ros2 topic echo /map               # View map data
```

### Check Nodes
```bash
ros2 node list                     # List all nodes
ros2 node info /cartographer_node  # Node information
```

### Visualize TF Tree
```bash
ros2 run tf2_tools view_frames     # Generate PDF of TF tree
```

### Clear Costmaps (if robot stuck)
```bash
ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntirelyGlobalCostmap
ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntirelyLocalCostmap
```

## Report Guidelines

Your 5-7 page report should include:

### 1. Introduction (0.5 pages)
- Project objectives
- Overview of approach

### 2. Maze Design (1 page)
- Maze dimensions and layout
- World file structure
- Screenshots of maze in Gazebo

### 3. SLAM Implementation (1 page)
- Cartographer configuration
- Mapping process
- Map quality assessment
- Screenshots of mapping process

### 4. Navigation Implementation (2 pages)

#### Nav2 Approach:
- A* algorithm explanation
- Parameter tuning
- Costmap configuration

#### Deep Q-Learning Approach:
- DQL architecture
- Reward function design
- Training process
- Convergence analysis

### 5. Results (1.5 pages)
- Performance comparison table
- Path visualizations
- Time efficiency analysis
- Success rates

### 6. Challenges & Solutions (0.5 pages)
- Technical difficulties encountered
- How you resolved them

### 7. Conclusion & Future Work (0.5 pages)
- Key findings
- Possible improvements
- Advanced techniques to explore

## Tips for VIVA Demonstration

1. **Prepare 3 diverse goal locations** beforehand
2. **Practice the demo** multiple times
3. **Have backup terminal commands** ready
4. **Explain while demonstrating:**
   - How SLAM works
   - How path planning works
   - Why robot makes certain decisions
5. **Be ready to answer:**
   - Why use A* vs Dijkstra?
   - How does AMCL localization work?
   - What are costmaps?
   - How does DQL learn?

## Additional Resources

- [ROS 2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [Nav2 Documentation](https://navigation.ros.org/)
- [Cartographer Documentation](https://google-cartographer-ros.readthedocs.io/)
- [TurtleBot3 Manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/)
- [Gazebo Harmonic Documentation](https://gazebosim.org/docs/harmonic)

## License

Apache License 2.0

## Authors

SLIIT - IE4060 Robotics and Intelligent Systems
Assignment 02 - Group Assignment

---

**Good luck with your project! 🤖**

