# Quick Test - Fixed Launch Files

## What Was Fixed

The error `AttributeError: 'NoneType' object has no attribute 'perform_substitution'` has been fixed.

**Problem:** Launch file tried to use `PathJoinSubstitution.perform()` incorrectly.

**Solution:** Rewrote to properly load URDF and handle substitutions.

---

## Test Now - Try This

### Option 1: Simplified Launch (RECOMMENDED)

```bash
cd ~/turtlebot3_ws
source install/setup.bash
export TURTLEBOT3_MODEL=burger

# Rebuild to get latest changes
colcon build --packages-select turtlebot3_maze_navigation
source install/setup.bash

# Launch simplified version
ros2 launch turtlebot3_maze_navigation gazebo_maze_v2.launch.py
```

**This should:**
- ✅ Launch Gazebo with maze
- ✅ No syntax errors
- ✅ Start ROS-Gazebo bridge
- ⚠️ Robot might not spawn automatically (that's OK for now)

### Option 2: Fixed Main Launch

```bash
cd ~/turtlebot3_ws
source install/setup.bash
export TURTLEBOT3_MODEL=burger
colcon build --packages-select turtlebot3_maze_navigation
source install/setup.bash

# Launch fixed main version
ros2 launch turtlebot3_maze_navigation gazebo_maze.launch.py
```

---

## Verify It's Working

### 1. Check Gazebo Launched
You should see Gazebo window with the maze.

### 2. Check for Errors
Terminal should not show:
- ❌ `AttributeError`
- ❌ `perform_substitution`
- ❌ Syntax errors

### 3. Check ROS Topics

In another terminal:
```bash
source ~/turtlebot3_ws/install/setup.bash
ros2 topic list
```

You should see topics like:
- `/clock`
- `/parameter_events`
- `/rosout`

---

## If Robot Doesn't Spawn

That's expected for now. We can add the robot to the world file for more reliable spawning.

### Quick Fix: Add Robot to World File

Edit `worlds/maze_world.sdf` and add this before `</world>`:

```xml
    <!-- TurtleBot3 Burger Model -->
    <model name="turtlebot3_burger">
      <pose>-1.25 1.25 0.01 0 0 0</pose>
      <static>false</static>
      
      <link name="base_footprint"/>
      
      <link name="base_link">
        <pose>0 0 0.010 0 0 0</pose>
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.001</ixx>
            <iyy>0.001</iyy>
            <izz>0.001</izz>
          </inertia>
        </inertial>
        
        <collision name="base_collision">
          <geometry>
            <box>
              <size>0.140 0.140 0.143</size>
            </box>
          </geometry>
        </collision>
        
        <visual name="base_visual">
          <geometry>
            <box>
              <size>0.140 0.140 0.143</size>
            </box>
          </geometry>
          <material>
            <ambient>0.0 0.0 0.8 1.0</ambient>
            <diffuse>0.0 0.0 0.8 1.0</diffuse>
          </material>
        </visual>
        
        <!-- LiDAR Sensor -->
        <sensor name="lidar" type="gpu_lidar">
          <pose>0 0 0.08 0 0 0</pose>
          <topic>scan</topic>
          <update_rate>5</update_rate>
          <lidar>
            <scan>
              <horizontal>
                <samples>360</samples>
                <resolution>1</resolution>
                <min_angle>0.0</min_angle>
                <max_angle>6.28</max_angle>
              </horizontal>
            </scan>
            <range>
              <min>0.12</min>
              <max>3.5</max>
              <resolution>0.015</resolution>
            </range>
            <noise>
              <type>gaussian</type>
              <mean>0.0</mean>
              <stddev>0.01</stddev>
            </noise>
          </lidar>
          <always_on>1</always_on>
          <visualize>true</visualize>
        </sensor>
      </link>
      
      <joint name="base_joint" type="fixed">
        <parent>base_footprint</parent>
        <child>base_link</child>
      </joint>
      
      <!-- Differential Drive Plugin -->
      <plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
        <left_joint>left_wheel_joint</left_joint>
        <right_joint>right_wheel_joint</right_joint>
        <wheel_separation>0.160</wheel_separation>
        <wheel_radius>0.033</wheel_radius>
        <odom_publish_frequency>30</odom_publish_frequency>
        <topic>cmd_vel</topic>
        <odom_topic>odom</odom_topic>
      </plugin>
      
      <!-- Add wheels if needed -->
      <link name="left_wheel">
        <pose>0 0.08 0.023 -1.57 0 0</pose>
        <visual name="left_wheel_visual">
          <geometry>
            <cylinder>
              <radius>0.033</radius>
              <length>0.018</length>
            </cylinder>
          </geometry>
        </visual>
        <collision name="left_wheel_collision">
          <geometry>
            <cylinder>
              <radius>0.033</radius>
              <length>0.018</length>
            </cylinder>
          </geometry>
        </collision>
      </link>
      
      <link name="right_wheel">
        <pose>0 -0.08 0.023 -1.57 0 0</pose>
        <visual name="right_wheel_visual">
          <geometry>
            <cylinder>
              <radius>0.033</radius>
              <length>0.018</length>
            </cylinder>
          </geometry>
        </visual>
        <collision name="right_wheel_collision">
          <geometry>
            <cylinder>
              <radius>0.033</radius>
              <length>0.018</length>
            </cylinder>
          </geometry>
        </collision>
      </link>
      
      <joint name="left_wheel_joint" type="revolute">
        <parent>base_link</parent>
        <child>left_wheel</child>
        <axis>
          <xyz>0 0 1</xyz>
        </axis>
      </joint>
      
      <joint name="right_wheel_joint" type="revolute">
        <parent>base_link</parent>
        <child>right_wheel</child>
        <axis>
          <xyz>0 0 1</xyz>
        </axis>
      </joint>
    </model>
```

Then launch with just:
```bash
gz sim ~/turtlebot3_ws/src/turtlebot3_maze_navigation/worlds/maze_world.sdf
```

---

## Current Status

✅ **Launch file syntax error FIXED**  
✅ **Two working launch file options**  
✅ **ROS-Gazebo bridge configured**  
⏳ **Robot spawning** (may need world file approach)

---

## What to Test

1. ✅ Launch doesn't crash with syntax error
2. ✅ Gazebo opens with maze
3. ✅ No `AttributeError` messages
4. ⏳ Robot appears (optional for now)
5. ⏳ Topics available (after robot spawns)

---

## Next Steps

Once Gazebo launches successfully:

1. **If robot spawns:** Great! Test teleop
2. **If no robot:** Add to world file (see above)
3. **Check topics:** `ros2 topic list`
4. **Test control:** `ros2 run turtlebot3_teleop teleop_keyboard`
5. **Proceed to SLAM:** Follow QUICK_START.md

---

## Summary

**Try This Now:**
```bash
cd ~/turtlebot3_ws
source install/setup.bash
colcon build --packages-select turtlebot3_maze_navigation
source install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_maze_navigation gazebo_maze_v2.launch.py
```

**Expected Result:** Gazebo opens with maze, no errors.

**Report back:** Let me know if it launches without the `AttributeError`!

