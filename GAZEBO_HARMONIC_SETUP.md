# Gazebo Harmonic vs Gazebo Classic - Important Information

## What's the Difference?

Your project uses **Gazebo Harmonic** (the new Gazebo, formerly called Ignition), NOT the older Gazebo Classic.

### Gazebo Classic (Old)
- Command: `gazebo`
- ROS integration: `gazebo_ros` packages
- Used with ROS 1 and earlier ROS 2 versions

### Gazebo Harmonic (New) ✓ **We use this**
- Command: `gz sim`
- ROS integration: `ros_gz` packages
- Recommended for ROS 2 Jazzy
- Better performance, modern architecture

## Why This Matters

The original launch file I created had mixed approaches (using `gazebo_ros` with `gz sim`). This has been corrected.

### What Changed

**Before (Incorrect):**
```python
from gazebo_ros import ...
spawn_turtlebot = Node(
    package='gazebo_ros',  # ❌ Wrong for Gazebo Harmonic
    executable='spawn_entity.py',
    ...
)
```

**After (Correct):**
```python
from ros_gz_sim import ...
spawn_turtlebot = Node(
    package='ros_gz_sim',  # ✓ Correct for Gazebo Harmonic
    executable='create',
    ...
)

# Plus: ROS-Gazebo bridge for topics
bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=[
        '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
        '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
        '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
        ...
    ]
)
```

## Required Packages

Install these packages for proper Gazebo Harmonic integration:

```bash
# ROS-Gazebo bridge packages
sudo apt install ros-jazzy-ros-gz-sim
sudo apt install ros-jazzy-ros-gz-bridge
sudo apt install ros-jazzy-ros-gz-spawn-entity
sudo apt install ros-jazzy-ros-gz-interfaces

# Verify installation
ros2 pkg list | grep ros_gz
```

You should see:
- ros_gz_bridge
- ros_gz_interfaces
- ros_gz_sim
- ros_gz_image
- And more...

## How It Works

### 1. Gazebo Harmonic runs the simulation
```bash
gz sim -r world_file.sdf
```

### 2. ROS-Gazebo Bridge connects ROS 2 to Gazebo
The bridge translates between:
- ROS 2 topics (e.g., `/cmd_vel`)
- Gazebo topics (e.g., `gz.msgs.Twist`)

### 3. Your ROS 2 nodes communicate normally
Nav2, Cartographer, etc., don't need to know about Gazebo - they just use standard ROS 2 topics.

## Topic Bridging

Our launch file sets up these bridges:

| ROS 2 Topic | Message Type | Gazebo Topic |
|-------------|--------------|--------------|
| `/scan` | sensor_msgs/LaserScan | gz.msgs.LaserScan |
| `/cmd_vel` | geometry_msgs/Twist | gz.msgs.Twist |
| `/odom` | nav_msgs/Odometry | gz.msgs.Odometry |
| `/tf` | tf2_msgs/TFMessage | gz.msgs.Pose_V |
| `/clock` | rosgraph_msgs/Clock | gz.msgs.Clock |

## Troubleshooting

### Issue: "Package 'ros_gz_sim' not found"

**Solution:**
```bash
sudo apt install ros-jazzy-ros-gz-sim
source /opt/ros/jazzy/setup.bash
```

### Issue: Topics not appearing in ROS 2

**Check if bridge is running:**
```bash
ros2 node list | grep bridge
```

**Check Gazebo topics:**
```bash
gz topic -l
```

**Check ROS 2 topics:**
```bash
ros2 topic list
```

### Issue: Robot not spawning

**Option 1:** Check if `turtlebot3_description` package is installed:
```bash
ros2 pkg list | grep turtlebot3_description
```

**Option 2:** Use alternative spawn method (include robot in world SDF).

## Alternative Approach: Include Robot in World File

If spawning is problematic, you can include the TurtleBot3 directly in the world SDF file:

```xml
<sdf version="1.8">
  <world name="maze_world">
    <!-- ... existing world content ... -->
    
    <!-- Include TurtleBot3 model -->
    <include>
      <uri>model://turtlebot3_burger</uri>
      <pose>-1.25 1.25 0.01 0 0 0</pose>
      <name>turtlebot3</name>
    </include>
    
  </world>
</sdf>
```

This requires:
1. TurtleBot3 model in Gazebo model path
2. Setting `GZ_SIM_RESOURCE_PATH` environment variable

## Checking Your Setup

Run this verification script:

```bash
# Check Gazebo version
gz sim --version

# Should show: Gazebo Sim, version 8.x.x (Harmonic)

# Check ros_gz packages
ros2 pkg list | grep ros_gz

# Should show multiple ros_gz packages

# Check if bridge executable exists
ros2 pkg executables ros_gz_bridge

# Should show: ros_gz_bridge parameter_bridge
```

## For Your Assignment

Everything has been corrected to use Gazebo Harmonic properly. Just make sure to:

1. Install the `ros_gz` packages as shown above
2. Use the updated launch files
3. Verify Gazebo Harmonic is installed: `gz sim --version`

## References

- [Gazebo Harmonic Documentation](https://gazebosim.org/docs/harmonic)
- [ros_gz Bridge Documentation](https://github.com/gazebosim/ros_gz)
- [ROS 2 Jazzy with Gazebo](https://docs.ros.org/en/jazzy/How-To-Guides/Using-Gazebo-Simulator.html)

## Summary

✅ **Correct Setup (What we now have):**
- Gazebo Harmonic (`gz sim`)
- `ros_gz_sim` for spawning
- `ros_gz_bridge` for topic bridging
- Proper SDF world file

❌ **Previous Issue (Fixed):**
- Mixed Gazebo Classic packages with Gazebo Harmonic commands
- Would have caused compatibility issues

Thank you for catching this! The project is now properly configured for Gazebo Harmonic with ROS 2 Jazzy.

