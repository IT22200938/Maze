# TurtleBot3 Autonomous Navigation in Maze Environment

**Course:** IE4060 – Robotics and Intelligent Systems  
**Assignment:** 02 – Group Assignment  
**Date:** [Insert Date]  
**Group Members:**
- Student 1 Name (ID)
- Student 2 Name (ID)
- Student 3 Name (ID)
- Student 4 Name (ID)

---

## Abstract

[Write a brief summary (150-200 words) covering: the problem, your approach, key results, and main findings]

---

## 1. Introduction (0.5 pages)

### 1.1 Project Objectives

The primary objective of this project is to implement autonomous navigation for a TurtleBot3 Burger robot in a maze environment using ROS 2 Jazzy and Gazebo simulator. The specific goals include:

- Designing and implementing a 6×6 maze environment in Gazebo
- Creating a map of the environment using SLAM (Simultaneous Localization and Mapping)
- Implementing autonomous navigation using Nav2 stack
- Developing a Deep Q-Learning approach for comparison
- Evaluating and comparing the performance of different navigation methods

### 1.2 System Overview

[Describe your overall system architecture, tools used, and approach]

- **Simulation Platform:** Gazebo Harmonic
- **Robot Platform:** TurtleBot3 Burger
- **ROS Version:** ROS 2 Jazzy Jalisco
- **Sensors:** LiDAR (360-degree laser scanner)
- **Navigation Stack:** Nav2
- **SLAM Method:** Cartographer

---

## 2. Maze Design and Implementation (1 page)

### 2.1 Maze Specifications

The maze was designed according to the following specifications:
- **Grid Size:** 6×6 cells
- **Cell Dimensions:** 0.5m × 0.5m each
- **Total Dimensions:** 3m × 3m
- **Wall Height:** 0.25m
- **Wall Thickness:** 0.05m

### 2.2 Gazebo World File Implementation

[Explain how you created the maze in Gazebo]

The maze was implemented using Gazebo's SDF (Simulation Description Format). Key implementation details:

```xml
<!-- Example of wall implementation -->
<model name="wall_example">
  <pose>x y z roll pitch yaw</pose>
  <static>true</static>
  <!-- collision and visual elements -->
</model>
```

### 2.3 Maze Layout

[Include a diagram/image of your maze layout]

![Maze Layout](path/to/maze_image.png)

The maze includes:
- Multiple pathways with varying complexity
- Dead ends to challenge navigation algorithms
- Open areas for navigation flexibility

### 2.4 Challenges in Maze Creation

[Describe any challenges faced and how you overcame them]

---

## 3. SLAM Implementation (1 page)

### 3.1 Cartographer Configuration

We used Google Cartographer for SLAM, configured with the following key parameters:

```lua
TRAJECTORY_BUILDER_2D.min_range = 0.12
TRAJECTORY_BUILDER_2D.max_range = 3.5
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
```

### 3.2 Mapping Process

**Steps taken:**
1. Launch Gazebo simulation with maze
2. Start Cartographer SLAM node
3. Manually navigate robot through entire maze
4. Save generated map

**Duration:** [X] minutes  
**Coverage:** [Y]% of maze area

### 3.3 Map Quality Assessment

[Include images comparing the actual maze to the generated map]

![Generated Map](path/to/map_image.png)

**Observations:**
- Walls are clearly defined: [Yes/No]
- Corridors are accurately represented: [Yes/No]
- Map resolution: [X] meters/pixel
- Map quality score: [assessment]

### 3.4 Issues and Solutions

[Describe any mapping issues encountered]

**Issue 1:** [Description]  
**Solution:** [How you fixed it]

---

## 4. Navigation Implementation (2 pages)

### 4.1 Nav2 Stack Configuration

#### 4.1.1 Path Planning Algorithm

We implemented A* (A-star) algorithm through Nav2's NavFn planner:

```yaml
planner_plugins: ["GridBased"]
GridBased:
  plugin: "nav2_navfn_planner/NavfnPlanner"
  use_astar: true
  tolerance: 0.5
```

**Why A* over Dijkstra?**
- A* uses heuristics for more efficient path finding
- Faster computation in known environments
- Optimal paths with admissible heuristics

#### 4.1.2 Local Planner (DWB)

Dynamic Window Approach (DWB) parameters:

```yaml
max_vel_x: 0.22
max_vel_theta: 2.0
sim_time: 1.7
```

#### 4.1.3 Costmap Configuration

**Global Costmap:**
- Uses static map layer
- Inflation radius: 0.55m
- Resolution: 0.05m

**Local Costmap:**
- Rolling window: 3m × 3m
- Real-time obstacle detection from LiDAR
- Updates at 5Hz

### 4.2 Deep Q-Learning Implementation

#### 4.2.1 Network Architecture

```
Input Layer:  24 neurons (LiDAR readings)
Hidden Layer 1: 128 neurons (ReLU)
Hidden Layer 2: 128 neurons (ReLU)
Hidden Layer 3: 64 neurons (ReLU)
Output Layer: 5 neurons (Actions)
```

#### 4.2.2 State Space

State representation includes:
- 24 LiDAR distance readings (sampled every 15°)
- Distance to goal
- Angle to goal

#### 4.2.3 Action Space

Five discrete actions:
1. Move forward
2. Move forward + turn left
3. Move forward + turn right
4. Sharp turn left
5. Sharp turn right

#### 4.2.4 Reward Function

```python
Reward = {
  -100  if collision
  +200  if goal reached
  +10*(previous_distance - current_distance)  # progress toward goal
  -0.1  per step  # encourage efficiency
}
```

#### 4.2.5 Training Parameters

- Learning rate: 0.001
- Discount factor (γ): 0.99
- Epsilon (initial): 1.0
- Epsilon (final): 0.01
- Epsilon decay: 0.995
- Batch size: 64
- Memory size: 2000 experiences
- Episodes trained: [X]

#### 4.2.6 Training Process

[Describe the training process and convergence]

![Training Curve](path/to/training_curve.png)

---

## 5. Experimental Results (1.5 pages)

### 5.1 Experimental Setup

**Test Goals:**
- Goal 1: Short distance (adjacent corridor) - Coordinates: (x, y)
- Goal 2: Medium distance (center of maze) - Coordinates: (x, y)
- Goal 3: Long distance (opposite corner) - Coordinates: (x, y)

**Metrics Recorded:**
- Time to goal (seconds)
- Path length (planned vs actual)
- Success rate
- Number of collisions

### 5.2 Nav2 Performance

| Goal | Time (s) | Planned Path (m) | Actual Path (m) | Efficiency (%) | Success | Collisions |
|------|----------|------------------|-----------------|----------------|---------|------------|
| 1    | X.XX     | X.XX            | X.XX            | XX.X           | ✓       | 0          |
| 2    | X.XX     | X.XX            | X.XX            | XX.X           | ✓       | 0          |
| 3    | X.XX     | X.XX            | X.XX            | XX.X           | ✓       | 0          |
| Avg  | X.XX     | X.XX            | X.XX            | XX.X           | 100%    | 0          |

### 5.3 Deep Q-Learning Performance

| Goal | Time (s) | Actual Path (m) | Success | Collisions | Episodes |
|------|----------|-----------------|---------|------------|----------|
| 1    | X.XX     | X.XX           | ✓       | X          | XXX      |
| 2    | X.XX     | X.XX           | ✓       | X          | XXX      |
| 3    | X.XX     | X.XX           | ✓       | X          | XXX      |
| Avg  | X.XX     | X.XX           | XX%     | X.X        | XXX      |

### 5.4 Comparative Analysis

#### 5.4.1 Performance Comparison

| Metric                | Nav2 (A*) | DQL      | Winner |
|-----------------------|-----------|----------|--------|
| Average Time          | X.XX s    | X.XX s   | [Nav2/DQL] |
| Average Path Length   | X.XX m    | X.XX m   | [Nav2/DQL] |
| Success Rate          | XX%       | XX%      | [Nav2/DQL] |
| Path Efficiency       | XX%       | XX%      | [Nav2/DQL] |
| Computational Cost    | Low       | High     | Nav2   |

#### 5.4.2 Path Visualizations

[Include screenshots or diagrams of paths taken by each method]

![Nav2 Path Example](path/to/nav2_path.png)  
*Figure X: Nav2 navigation path for Goal 2*

![DQL Path Example](path/to/dql_path.png)  
*Figure X: DQL navigation path for Goal 2*

#### 5.4.3 Discussion

**Nav2 Advantages:**
- [List advantages observed]
- Deterministic and predictable
- Fast computation
- Optimal paths with complete map

**Nav2 Disadvantages:**
- [List disadvantages]
- Requires accurate map
- Limited dynamic adaptation

**DQL Advantages:**
- [List advantages]
- Learns from experience
- Potential for better adaptation

**DQL Disadvantages:**
- [List disadvantages]
- Long training time
- Less predictable
- May not converge to optimal

---

## 6. Challenges and Solutions (0.5 pages)

### 6.1 Technical Challenges

**Challenge 1: [Description]**
- Problem: [Detailed description]
- Impact: [How it affected the project]
- Solution: [How you resolved it]
- Outcome: [Result of the solution]

**Challenge 2: [Description]**
- Problem: [Detailed description]
- Impact: [How it affected the project]
- Solution: [How you resolved it]
- Outcome: [Result of the solution]

**Challenge 3: [Description]**
- Problem: [Detailed description]
- Impact: [How it affected the project]
- Solution: [How you resolved it]
- Outcome: [Result of the solution]

### 6.2 Lessons Learned

[Discuss key takeaways from overcoming these challenges]

---

## 7. Conclusion and Future Work (0.5 pages)

### 7.1 Key Findings

1. [Finding 1]
2. [Finding 2]
3. [Finding 3]

### 7.2 Project Success

The project successfully achieved its objectives:
- ✓ Created functional 6×6 maze in Gazebo
- ✓ Implemented SLAM mapping with Cartographer
- ✓ Implemented Nav2 autonomous navigation
- ✓ Implemented Deep Q-Learning approach
- ✓ Compared performance of both methods

### 7.3 Future Improvements

**Short-term:**
1. Fine-tune DQL reward function for better performance
2. Implement more sophisticated obstacle avoidance
3. Test with dynamic obstacles

**Long-term:**
1. Implement other RL algorithms (PPO, SAC)
2. Multi-goal path planning
3. Dynamic environment adaptation
4. Transfer learning to real robot

### 7.4 Real-World Applications

The techniques demonstrated in this project have applications in:
- Warehouse automation
- Hospital delivery robots
- Search and rescue operations
- Autonomous vehicles in structured environments

---

## References

1. ROS 2 Documentation. (2024). *ROS 2 Jazzy Jalisco*. Retrieved from https://docs.ros.org/en/jazzy/
2. Macenski, S., et al. (2020). "The Marathon 2: A Navigation System." *IEEE/RSJ IROS*.
3. Hess, W., et al. (2016). "Real-time loop closure in 2D LIDAR SLAM." *IEEE ICRA*.
4. Mnih, V., et al. (2015). "Human-level control through deep reinforcement learning." *Nature*.
5. TurtleBot3 Documentation. (2024). *ROBOTIS e-Manual*. Retrieved from https://emanual.robotis.com/

---

## Appendices

### Appendix A: Source Code Repository

GitHub Repository: [Your GitHub URL]

### Appendix B: Launch Commands

```bash
# Gazebo simulation
ros2 launch turtlebot3_maze_navigation gazebo_maze.launch.py

# SLAM mapping
ros2 launch turtlebot3_maze_navigation cartographer.launch.py

# Navigation
ros2 launch turtlebot3_maze_navigation navigation.launch.py
```

### Appendix C: Configuration Files

[Include key configuration file snippets if needed]

### Appendix D: Additional Results

[Any additional graphs, tables, or data]

---

**End of Report**

