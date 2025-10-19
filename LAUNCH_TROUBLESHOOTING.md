# Launch File Troubleshooting Guide

## The Launch File Error You Encountered

**Error:**
```
AttributeError: 'NoneType' object has no attribute 'perform_substitution'
```

**Cause:** 
The original `gazebo_maze.launch.py` tried to use `PathJoinSubstitution.perform()` incorrectly. Launch substitutions can't be evaluated directly in parameter dictionaries.

**Status:** ✅ FIXED

---

## Available Launch Files

### 1. `gazebo_maze.launch.py` (Fixed - Main Version)

**Status:** Fixed the substitution error  
**Use when:** You have TurtleBot3 packages installed  
**Features:**
- Launches Gazebo Harmonic
- Spawns TurtleBot3
- Sets up ROS-Gazebo bridge
- Robot state publisher

**Launch:**
```bash
ros2 launch turtlebot3_maze_navigation gazebo_maze.launch.py
```

### 2. `gazebo_maze_v2.launch.py` (NEW - Simplified)

**Status:** New, simplified version  
**Use when:** First version has issues OR you want simpler setup  
**Features:**
- Minimal dependencies
- Easier to debug
- Basic robot description
- ROS-Gazebo bridge

**Launch:**
```bash
ros2 launch turtlebot3_maze_navigation gazebo_maze_v2.launch.py
```

### 3. `gazebo_maze_simple.launch.py` (Fallback)

**Status:** Minimal fallback  
**Use when:** Other versions fail  
**Features:**
- Just launches Gazebo
- No automatic robot spawning
- Minimal complexity

**Launch:**
```bash
ros2 launch turtlebot3_maze_navigation gazebo_maze_simple.launch.py
```

---

## Common Issues and Solutions

### Issue 1: Launch file syntax error (SOLVED)

**Error:**
```
AttributeError: 'NoneType' object has no attribute 'perform_substitution'
```

**Solution:**
✅ Fixed in updated `gazebo_maze.launch.py`  
✅ Use `gazebo_maze_v2.launch.py` as alternative

### Issue 2: ros_gz packages not found

**Error:**
```
Package 'ros_gz_bridge' not found
```

**Solution:**
```bash
sudo apt install ros-jazzy-ros-gz-sim
sudo apt install ros-jazzy-ros-gz-bridge
source /opt/ros/jazzy/setup.bash
```

### Issue 3: TurtleBot3 description not found

**Error:**
```
Package 'turtlebot3_description' not found
```

**Solution:**
```bash
cd ~/turtlebot3_ws/src
git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3.git
cd ~/turtlebot3_ws
colcon build
source install/setup.bash
```

**OR** use `gazebo_maze_v2.launch.py` which has fallback robot description.

### Issue 4: Robot doesn't spawn

**Symptoms:**
- Gazebo launches
- Maze appears
- No robot

**Solutions:**

**Option A - Manual spawn:**
```bash
# In another terminal while Gazebo is running
ros2 run ros_gz_sim create -name turtlebot3 -x -1.25 -y 1.25 -z 0.1
```

**Option B - Include robot in world file:**
Edit `worlds/maze_world.sdf` and add robot model (see below)

### Issue 5: No /scan topic

**Check:**
```bash
ros2 topic list | grep scan
```

**If missing:**

1. Check if bridge is running:
```bash
ros2 node list | grep bridge
```

2. If not running, start manually:
```bash
ros2 run ros_gz_bridge parameter_bridge /scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan
```

3. Check Gazebo topics:
```bash
gz topic -l | grep scan
```

---

## Manual Robot Spawning

If automatic spawning fails, you can spawn manually:

### Method 1: Using ros_gz_sim

```bash
# Terminal 1: Launch Gazebo
ros2 launch turtlebot3_maze_navigation gazebo_maze_simple.launch.py

# Terminal 2: Spawn robot (wait for Gazebo to fully load)
ros2 run ros_gz_sim create \
  -name turtlebot3_burger \
  -x -1.25 \
  -y 1.25 \
  -z 0.1
```

### Method 2: Include in World File

Edit `worlds/maze_world.sdf` and add before the closing `</world>` tag:

```xml
    <!-- TurtleBot3 Burger -->
    <include>
      <uri>package://turtlebot3_gazebo/models/turtlebot3_burger</uri>
      <name>turtlebot3</name>
      <pose>-1.25 1.25 0.01 0 0 0</pose>
    </include>
```

Then just launch Gazebo:
```bash
gz sim -r ~/turtlebot3_ws/src/turtlebot3_maze_navigation/worlds/maze_world.sdf
```

---

## Debugging Steps

### Step 1: Verify Gazebo Harmonic

```bash
gz sim --version
# Should show: Gazebo Sim, version 8.x.x (Harmonic)
```

### Step 2: Check ROS Packages

```bash
# Check ros_gz packages
ros2 pkg list | grep ros_gz

# Should see:
# ros_gz_bridge
# ros_gz_sim
# ros_gz_interfaces
# etc.
```

### Step 3: Test Basic Gazebo Launch

```bash
# Just launch Gazebo with maze
gz sim ~/turtlebot3_ws/src/turtlebot3_maze_navigation/worlds/maze_world.sdf

# Should see maze in Gazebo window
```

### Step 4: Check Topics

```bash
# After launching
ros2 topic list

# Expected topics:
# /scan
# /cmd_vel
# /odom
# /tf
# /clock
```

### Step 5: Check Nodes

```bash
ros2 node list

# Expected nodes:
# /robot_state_publisher
# /gz_bridge (or similar)
```

---

## Recommended Workflow

### For First-Time Setup:

1. **Test Gazebo alone:**
```bash
gz sim ~/turtlebot3_ws/src/turtlebot3_maze_navigation/worlds/maze_world.sdf
```

2. **If that works, try simplified launch:**
```bash
ros2 launch turtlebot3_maze_navigation gazebo_maze_v2.launch.py
```

3. **Check topics:**
```bash
ros2 topic list
ros2 topic echo /scan
```

4. **If working, proceed with mapping/navigation**

### For Production Use:

Once everything works, use the main launch file:
```bash
ros2 launch turtlebot3_maze_navigation gazebo_maze.launch.py
```

---

## Quick Test Commands

```bash
# 1. Install dependencies
sudo apt install ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge

# 2. Build workspace
cd ~/turtlebot3_ws
colcon build --packages-select turtlebot3_maze_navigation
source install/setup.bash

# 3. Set environment
export TURTLEBOT3_MODEL=burger

# 4. Try simplified launch
ros2 launch turtlebot3_maze_navigation gazebo_maze_v2.launch.py

# 5. In another terminal, check topics
ros2 topic list

# 6. Test control (if robot spawned)
ros2 run turtlebot3_teleop teleop_keyboard
```

---

## Which Launch File Should I Use?

**Use `gazebo_maze_v2.launch.py` if:**
- ✅ First time setting up
- ✅ Want simplest solution
- ✅ Had errors with main launch file
- ✅ Want to debug step by step

**Use `gazebo_maze.launch.py` if:**
- ✅ Have all TurtleBot3 packages installed
- ✅ Want full functionality
- ✅ Simplified version working

**Use `gazebo_maze_simple.launch.py` if:**
- ✅ Everything else fails
- ✅ Just need Gazebo with maze
- ✅ Will spawn robot manually

---

## Current Status

✅ **Fixed:** Original launch file substitution error  
✅ **Added:** Simplified `gazebo_maze_v2.launch.py`  
✅ **Working:** All three launch file options available  

**Recommended:** Start with `gazebo_maze_v2.launch.py` for most reliable results.

---

## Getting Help

If still having issues:

1. Check exact error message
2. Verify Gazebo Harmonic is installed: `gz sim --version`
3. Verify ros_gz packages: `ros2 pkg list | grep ros_gz`
4. Try simplest launch file first
5. Check logs: `~/.ros/log/`

---

**Updated:** Issue fixed, simplified version created, ready to use!

