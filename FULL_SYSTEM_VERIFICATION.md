# Full System Launch Verification

## Command
```bash
ros2 launch turtlebot3_autonav full_sim_launch.py
```

## Expected System Behavior

### Timeline of Events

#### Phase 1: Gazebo Initialization (0-5 seconds)
- **t=0s**: Gazebo server starts with maze.world
- **t=3s**: Gazebo client (GUI) starts
- **Expected Result**: Gazebo window opens showing the 3m×3m maze environment

#### Phase 2: Robot Setup (5-8 seconds)  
- **t=0s**: Robot state publisher starts (publishes robot URDF/transforms)
- **t=5s**: Robot spawns at position **(0.5, 0.5, 0.01)** ⚠️ **NEEDS FIX**
- **t=5s**: ROS-Gazebo bridge starts
  - Bridges topics: `/scan`, `/cmd_vel`, `/odom`, `/clock`
- **Expected Result**: Orange TurtleBot3 Burger appears in maze, topics are active

#### Phase 3: SLAM Initialization (8-10 seconds)
- **t=8s**: SLAM Toolbox (`async_slam_toolbox_node`) starts
  - Uses config: `config/slam_params.yaml`
  - Subscribes to: `/scan`
  - Publishes: `/map`, `/map_metadata`
- **Expected Result**: Map building begins, robot starts creating occupancy grid

#### Phase 4: Navigation Stack (10-12 seconds)
- **t=10s**: Nav2 stack launches
  - Map Server
  - AMCL (localization)
  - Planner Server (global path planning)
  - Controller Server (local trajectory control)
  - Behavior Server
  - BT Navigator (behavior trees)
  - Lifecycle Manager
  - Uses config: `config/nav2_params.yaml`
- **Expected Result**: Navigation nodes active, ready to accept goal poses

#### Phase 5: UI & Safety Systems (12-15 seconds)
- **t=12s**: Goal Clicker Node starts
  - Allows clicking in RViz to set navigation goals
- **t=12s**: Obstacle Monitor starts (C++ node)
  - Safety distance: 0.18m
  - Monitors `/scan` for obstacles
  - Can emergency stop robot
- **t=12s**: Performance Recorder starts
  - Logs navigation metrics
  - Records success rate, path efficiency, etc.

#### Phase 6: Visualization (15+ seconds)
- **t=15s**: RViz2 opens
  - Config: `rviz/navigation.rviz`
  - Displays: robot model, map, laser scan, planned path, cost maps
- **Expected Result**: RViz window shows complete system visualization

---

## Issues Identified & Required Fixes

### 🔴 CRITICAL ISSUES

1. **Wrong Robot Spawn Position**
   - **File**: `launch/full_sim_launch.py` (lines 117-119)
   - **Current**: `x=0.5, y=0.5` 
   - **Should be**: `x=1.5, y=1.5` (center of maze)
   - **Impact**: Robot spawns at edge instead of center

2. **Broken Robot Description**
   - **File**: `launch/full_sim_launch.py` (lines 79-96)
   - **Issue**: Using `ExecuteProcess.output` directly doesn't work
   - **Fix**: Use `Command()` and `ParameterValue()` (same as gazebo_only.launch.py)
   - **Impact**: Robot may not spawn correctly

3. **Missing Python Executables**
   - **Files needed**: 
     - `scripts/ui/goal_clicker_node.py` 
     - `scripts/dqn_agent/dqn_node.py`
     - `scripts/tools/recorder_node.py`
   - **Issue**: Launch file expects these as executable nodes
   - **Impact**: Nodes won't start, system incomplete

4. **C++ Obstacle Monitor Not Built**
   - **File**: `src/safety/obstacle_monitor.cpp`
   - **Issue**: Needs to be compiled with CMakeLists.txt
   - **Impact**: Safety monitoring won't work

### ⚠️ CONFIGURATION ISSUES

5. **Missing/Incomplete Config Files**
   - `config/slam_params.yaml` - SLAM Toolbox configuration
   - `config/nav2_params.yaml` - Nav2 stack parameters
   - `config/dqn_params.yaml` - DQN agent settings
   - **Impact**: Nodes may fail or use incorrect defaults

6. **Missing RViz Config**
   - `rviz/navigation.rviz`
   - **Impact**: RViz won't show proper visualization layout

---

## Default Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `use_sim_time` | `True` | Use Gazebo clock |
| `world` | `world/maze.world` | Path to world file |
| `nav_mode` | `nav2` | Navigation mode (nav2/dqn) |
| `use_slam` | `True` | Enable SLAM mapping |
| `use_rviz` | `True` | Start RViz visualization |
| `enable_safety` | `True` | Enable obstacle monitor |
| `enable_recording` | `True` | Enable performance logging |

---

## System Requirements

### ROS 2 Packages Needed
- `ros_gz_sim` - Gazebo integration
- `ros_gz_bridge` - ROS-Gazebo topic bridging
- `slam_toolbox` - SLAM mapping
- `nav2_bringup` - Navigation stack
- `robot_state_publisher` - Robot transforms
- `rviz2` - Visualization

### Topics Expected
- `/scan` - LaserScan (360 samples)
- `/cmd_vel` - Twist commands
- `/odom` - Odometry
- `/map` - Occupancy grid
- `/goal_pose` - Navigation goals
- `/clock` - Simulation time

### Transforms Expected
```
odom → base_footprint → base_link → lidar_link
       └→ wheel_left_link
       └→ wheel_right_link
       └→ caster_link
```

---

## Success Criteria

✅ **System is working correctly when:**
1. Gazebo opens with maze visible
2. Robot spawns at center (1.5, 1.5)
3. Robot is orange with blue LiDAR sensor
4. Green laser scan rays are visible
5. RViz opens showing map being built
6. `/scan`, `/odom`, `/cmd_vel` topics are publishing
7. No critical errors in terminal
8. Can click "2D Goal Pose" in RViz to send robot to locations
9. Robot navigates around obstacles
10. Map gradually fills in as robot explores

---

## Quick Diagnostic Commands

```bash
# Check if all topics are active
ros2 topic list

# Verify robot description is published
ros2 topic echo /robot_description --once

# Check laser scan data
ros2 topic echo /scan --once

# Verify Nav2 is active
ros2 node list | grep nav2

# Check SLAM is running
ros2 node list | grep slam

# Monitor transforms
ros2 run tf2_tools view_frames
```

---

## Priority Fix Order

1. ✅ Fix robot spawn position (line 117)
2. ✅ Fix robot_state_publisher URDF processing (lines 85-96)
3. ⚠️ Ensure Python nodes are executable and in correct paths
4. ⚠️ Compile C++ obstacle_monitor 
5. ⚠️ Verify all config files exist and are valid
6. ⚠️ Test each phase individually before full system launch

