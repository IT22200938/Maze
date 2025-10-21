# System Architecture Documentation

## Overview

This document describes the detailed architecture of the TurtleBot3 Autonomous Navigation system.

## System Layers

### 1. Simulation Layer

**Components:**
- Gazebo Sim (version 8.9.0+)
- Custom maze world (`maze.world`)
- TurtleBot3 Burger robot model (URDF/Xacro)

**Responsibilities:**
- Physics simulation
- Sensor simulation (LiDAR)
- Robot dynamics
- Environment representation

**Published Topics:**
- `/scan` (sensor_msgs/LaserScan) - LiDAR data
- `/odom` (nav_msgs/Odometry) - Robot odometry
- `/joint_states` (sensor_msgs/JointState) - Robot joint states

**Subscribed Topics:**
- `/cmd_vel` (geometry_msgs/Twist) - Velocity commands

---

### 2. Perception Layer

**Components:**
- SLAM Toolbox (async_slam_toolbox_node)
- TF2 transform system
- Sensor data processing

**Responsibilities:**
- Real-time mapping
- Localization
- Map updates
- Coordinate frame transformations

**Published Topics:**
- `/map` (nav_msgs/OccupancyGrid) - 2D occupancy grid map
- `/tf` - Transform tree
- `/map_metadata` - Map information

**Subscribed Topics:**
- `/scan` - LiDAR data for mapping
- `/odom` - Odometry for localization

**Configuration:**
- `config/slam_params.yaml`

---

### 3. Navigation & Planning Layer

#### Option A: Nav2 Stack

**Components:**
- Planner Server (NavFn with A* algorithm)
- Controller Server (DWB local planner)
- Behavior Server (recovery behaviors)
- BT Navigator (behavior tree executor)
- Lifecycle Manager

**Responsibilities:**
- Global path planning
- Local trajectory optimization
- Obstacle avoidance
- Goal following
- Recovery behaviors

**Published Topics:**
- `/plan` (nav_msgs/Path) - Global plan
- `/local_plan` (nav_msgs/Path) - Local trajectory
- `/cmd_vel` - Velocity commands
- `/global_costmap/costmap` - Global costmap
- `/local_costmap/costmap` - Local costmap

**Subscribed Topics:**
- `/map` - Static map
- `/scan` - Obstacle detection
- `/odom` - Localization
- `/goal_pose` - Navigation goals

**Configuration:**
- `config/nav2_params.yaml`

#### Option B: DQN Agent

**Components:**
- DQN Neural Network (PyTorch)
- Replay Buffer
- Q-Learning algorithm
- State processor

**Responsibilities:**
- Learned navigation policy
- Action selection
- Experience replay
- Model training/inference

**Neural Network Architecture:**
```
Input Layer: state_size (26 dimensions)
  ├─ 24 LiDAR points (normalized)
  ├─ 1 goal distance (normalized)
  └─ 1 goal angle (normalized)
     ↓
Hidden Layer 1: 128 neurons (ReLU)
     ↓
Hidden Layer 2: 128 neurons (ReLU)
     ↓
Output Layer: 4 actions
  ├─ Action 0: Forward (v=0.22, ω=0.0)
  ├─ Action 1: Turn Left (v=0.1, ω=1.0)
  ├─ Action 2: Turn Right (v=0.1, ω=-1.0)
  └─ Action 3: Stop (v=0.0, ω=0.0)
```

**Published Topics:**
- `/cmd_vel` - Learned velocity commands

**Subscribed Topics:**
- `/scan` - LiDAR observations
- `/odom` - Robot state
- `/goal_pose` - Target position

**Configuration:**
- `config/dqn_params.yaml`

---

### 4. User Interaction Layer

**Components:**
- RViz2 (visualization)
- Goal Clicker Node (goal_clicker_node.py)

**Responsibilities:**
- Map visualization
- Goal setting interface
- Robot state display
- Path visualization

**Published Topics:**
- `/goal_pose` - User-selected goals
- `/initialpose` - Initial pose estimates

**Subscribed Topics:**
- `/map` - Map display
- `/scan` - LiDAR visualization
- `/plan` - Path display
- `/tf` - Transform visualization

---

### 5. Safety & Monitoring Layer

**Components:**
- Obstacle Monitor (obstacle_monitor.cpp)
- Emergency stop system

**Responsibilities:**
- Collision detection
- Emergency stop trigger
- Safety distance monitoring
- Velocity filtering

**Published Topics:**
- `/cmd_vel` - Filtered velocity commands
- `/emergency_stop` (std_msgs/Bool) - Emergency stop status

**Subscribed Topics:**
- `/cmd_vel_input` - Raw velocity commands
- `/scan` - Obstacle detection

**Parameters:**
- `safety_distance`: 0.18 m (configurable)
- `enable_safety`: true/false

---

### 6. Evaluation & Logging Layer

**Components:**
- Performance Recorder (recorder_node.py)
- Log file system

**Responsibilities:**
- Metric collection
- Performance logging
- Session tracking
- Data persistence

**Logged Metrics:**
- Duration (seconds)
- Distance traveled (meters)
- Path efficiency (%)
- Collision count
- Success rate
- Average speed (m/s)

**Output:**
- JSON log files in `navigation_logs/`
- Per-session logs
- Cumulative statistics

---

## Data Flow Diagram

```
┌─────────────┐
│   User      │
│  (RViz2)    │
└──────┬──────┘
       │ /goal_pose
       ↓
┌─────────────────────────────────┐
│  Goal Clicker Node              │
│  (goal_clicker_node.py)         │
└──────┬──────────────────────────┘
       │ /goal_pose
       ↓
┌─────────────────────────────────┐
│  Navigation Layer               │
│  ┌───────────┬──────────────┐   │
│  │  Nav2     │  DQN Agent   │   │
│  │  Stack    │  (optional)  │   │
│  └───────────┴──────────────┘   │
└──────┬──────────────────────────┘
       │ /cmd_vel_input
       ↓
┌─────────────────────────────────┐
│  Safety Monitor                 │
│  (obstacle_monitor.cpp)         │◄─── /scan
└──────┬──────────────────────────┘
       │ /cmd_vel (filtered)
       ↓
┌─────────────────────────────────┐
│  Robot (Gazebo Sim)             │
│  - Physics simulation           │
│  - Sensor simulation            │
└──────┬──────────────────────────┘
       │ /scan, /odom
       ↓
┌─────────────────────────────────┐
│  SLAM Toolbox                   │
│  - Mapping                      │
│  - Localization                 │
└──────┬──────────────────────────┘
       │ /map, /tf
       └──────┐
              │
       ┌──────▼──────────────────┐
       │  Performance Recorder   │
       │  (recorder_node.py)     │
       └─────────────────────────┘
```

## Communication Patterns

### Topic Communication

| Topic | Publisher | Subscriber(s) | Rate | QoS |
|-------|-----------|---------------|------|-----|
| `/scan` | Gazebo | SLAM, Nav2, DQN, Safety | 10 Hz | Best Effort |
| `/odom` | Gazebo | SLAM, Nav2, DQN, Recorder | 50 Hz | Reliable |
| `/map` | SLAM | Nav2, RViz | 1 Hz | Transient Local |
| `/goal_pose` | Goal Clicker | Nav2, DQN | Event | Reliable |
| `/cmd_vel` | Safety Monitor | Gazebo | 10 Hz | Reliable |
| `/plan` | Nav2 Planner | RViz, Recorder | 1 Hz | Reliable |

### Transform (TF) Tree

```
map
 └─ odom
     └─ base_footprint
         └─ base_link
             ├─ wheel_left_link
             ├─ wheel_right_link
             ├─ caster_link
             └─ lidar_link
```

**Frame Descriptions:**
- `map`: Fixed global reference frame (SLAM)
- `odom`: Odometry frame (continuous)
- `base_footprint`: Robot base projection on ground
- `base_link`: Robot center
- `lidar_link`: LiDAR sensor frame

## Node Dependencies

```
gazebo_sim
    ├─► slam_toolbox
    │       └─► nav2_bringup
    │               └─► goal_clicker
    ├─► obstacle_monitor
    └─► recorder_node

Alternative:
gazebo_sim
    ├─► slam_toolbox
    │       └─► dqn_agent
    │               └─► goal_clicker
    ├─► obstacle_monitor
    └─► recorder_node
```

## Launch Sequence

1. **T+0s**: Start Gazebo Sim with maze world
2. **T+3s**: Start Gazebo client (GUI)
3. **T+5s**: Spawn robot, start ROS-Gazebo bridge
4. **T+8s**: Start SLAM Toolbox
5. **T+10s**: Start Nav2 or DQN agent
6. **T+12s**: Start goal clicker, safety monitor, recorder
7. **T+15s**: Start RViz2

## Configuration Files

| File | Purpose |
|------|---------|
| `slam_params.yaml` | SLAM Toolbox configuration |
| `nav2_params.yaml` | Nav2 stack parameters |
| `dqn_params.yaml` | DQN agent hyperparameters |
| `turtlebot3_burger.urdf.xacro` | Robot description |
| `maze.world` | Gazebo world definition |
| `navigation.rviz` | RViz visualization setup |

## Performance Characteristics

### Nav2 Mode
- **Planning Time**: ~50-200ms
- **Control Frequency**: 10 Hz
- **Map Update Rate**: 1 Hz
- **Typical Success Rate**: 85-95%

### DQN Mode
- **Inference Time**: ~5-20ms (CPU), ~2-5ms (GPU)
- **Action Frequency**: 10 Hz
- **Training Episodes**: 100-500 for convergence
- **Typical Success Rate**: 60-80% (after training)

## Extensibility

### Adding New Navigation Algorithm

1. Create new node in `src/` or `scripts/`
2. Subscribe to `/scan`, `/odom`, `/map`, `/goal_pose`
3. Publish to `/cmd_vel_input`
4. Add to launch file with conditional execution

### Adding New Sensor

1. Update `urdf/turtlebot3_burger.urdf.xacro`
2. Add Gazebo plugin for sensor
3. Update SLAM/Nav2 configuration to use sensor data
4. Add RViz visualization

### Custom Reward Function (DQN)

Modify `calculate_reward()` in `scripts/dqn_agent/dqn_node.py`:

```python
def calculate_reward(self, state):
    # Custom reward logic here
    return reward
```

## References

- [Nav2 Architecture](https://navigation.ros.org/concepts/index.html)
- [SLAM Toolbox Documentation](https://github.com/SteveMacenski/slam_toolbox)
- [ROS 2 Design](https://design.ros2.org/)
- [Gazebo Architecture](https://gazebosim.org/docs)

