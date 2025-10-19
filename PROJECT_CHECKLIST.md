# TurtleBot3 Maze Navigation - Project Checklist

Use this checklist to track your progress through the project.

## Phase 1: Setup and Installation ✓

- [x] Install Ubuntu 24.04 LTS
- [x] Install ROS 2 Jazzy
- [x] Install Gazebo Harmonic
- [x] Install TurtleBot3 packages
- [x] Install Cartographer
- [x] Install Nav2
- [x] Clone/create project package
- [ ] Build package successfully
- [ ] Run test_setup.py - all checks pass

## Phase 2: Maze World Creation ✓

- [x] Design 6x6 maze layout (3m x 3m)
- [x] Create maze_world.sdf file
- [x] Implement outer boundary walls
- [x] Implement internal walls according to layout
- [x] Add lighting and physics
- [ ] Test maze in Gazebo
- [ ] Verify wall positions match design
- [ ] Confirm robot spawns correctly

## Phase 3: Basic Testing □

- [ ] Launch Gazebo with maze successfully
- [ ] Robot spawns without errors
- [ ] Test teleoperation
  - [ ] Forward movement works
  - [ ] Turning works
  - [ ] Robot doesn't fall through floor
  - [ ] LiDAR is publishing data (`ros2 topic echo /scan`)
- [ ] Drive robot manually through maze
- [ ] Verify no collision issues
- [ ] Take screenshots of maze

## Phase 4: SLAM Mapping □

- [ ] Launch Cartographer successfully
- [ ] RViz opens with correct configuration
- [ ] LiDAR visualization appears
- [ ] Map building starts
- [ ] Drive robot through entire maze
  - [ ] Cover all corridors
  - [ ] Visit all areas
  - [ ] Drive smoothly (not too fast)
- [ ] Save map successfully
  - [ ] maze_map.yaml created
  - [ ] maze_map.pgm created
- [ ] Verify map quality
  - [ ] All walls visible
  - [ ] Corridors clear
  - [ ] Minimal noise
- [ ] Take screenshots of mapping process
- [ ] Take screenshot of final map

## Phase 5: Nav2 Navigation □

- [ ] Launch Nav2 successfully
- [ ] Map loads correctly in RViz
- [ ] Set initial pose (2D Pose Estimate)
- [ ] AMCL localizes robot correctly
- [ ] Costmaps display correctly
  - [ ] Global costmap visible
  - [ ] Local costmap visible
  - [ ] Inflation layers correct
- [ ] Test navigation to Goal 1 (short distance)
  - [ ] Plan path using 2D Goal Pose
  - [ ] Robot navigates successfully
  - [ ] Record time
  - [ ] Record path length
  - [ ] Take screenshot/video
- [ ] Test navigation to Goal 2 (medium distance)
  - [ ] Plan path
  - [ ] Navigate successfully
  - [ ] Record metrics
  - [ ] Take screenshot/video
- [ ] Test navigation to Goal 3 (long distance)
  - [ ] Plan path
  - [ ] Navigate successfully
  - [ ] Record metrics
  - [ ] Take screenshot/video
- [ ] Record performance data
- [ ] Test at least 3 additional goals

## Phase 6: Deep Q-Learning (Optional/Advanced) □

- [ ] Install PyTorch (`pip3 install torch`)
- [ ] Test DQL script runs without errors
- [ ] Configure DQL parameters
- [ ] Start training
  - [ ] Monitor episodes
  - [ ] Check reward progression
  - [ ] Save model periodically
- [ ] Train for sufficient episodes ([your target])
- [ ] Test trained model
  - [ ] Goal 1 navigation
  - [ ] Goal 2 navigation
  - [ ] Goal 3 navigation
- [ ] Record DQL performance metrics
- [ ] Compare with Nav2 results

## Phase 7: Performance Analysis □

- [ ] Run performance_tracker.py
- [ ] Collect data for Nav2:
  - [ ] 3 goals × multiple runs
  - [ ] Record all metrics
- [ ] Collect data for DQL:
  - [ ] 3 goals × multiple runs
  - [ ] Record all metrics
- [ ] Create comparison table
- [ ] Calculate averages
- [ ] Analyze path efficiency
- [ ] Identify strengths/weaknesses of each method
- [ ] Create visualization graphs

## Phase 8: Documentation □

- [ ] Fill out REPORT_TEMPLATE.md
  - [ ] Abstract
  - [ ] Introduction
  - [ ] Maze design section
  - [ ] SLAM section
  - [ ] Navigation section
  - [ ] Results section
  - [ ] Challenges section
  - [ ] Conclusion
- [ ] Include all required images:
  - [ ] Maze diagram
  - [ ] Generated map
  - [ ] Path visualizations
  - [ ] Training curves (if DQL)
- [ ] Complete all tables with real data
- [ ] Proofread entire report
- [ ] Check page count (5-7 pages)
- [ ] Format references correctly
- [ ] Export to PDF

## Phase 9: Code Repository □

- [ ] Create GitHub repository
- [ ] Push all code
- [ ] Include README.md
- [ ] Add .gitignore
- [ ] Verify all launch files included
- [ ] Verify all config files included
- [ ] Add proper documentation
- [ ] Test cloning and building from fresh checkout
- [ ] Share repository link in report

## Phase 10: VIVA Preparation □

- [ ] Practice demo multiple times
- [ ] Prepare 3 demo goal locations
- [ ] Test all commands work
- [ ] Prepare explanation of:
  - [ ] SLAM process
  - [ ] A* algorithm
  - [ ] Costmaps
  - [ ] DQL (if implemented)
- [ ] Anticipate questions:
  - [ ] Why A* vs Dijkstra?
  - [ ] How does AMCL work?
  - [ ] What challenges did you face?
  - [ ] How could performance be improved?
- [ ] Prepare backup plan if demo fails
- [ ] Bring printed report copy
- [ ] Bring laptop with everything set up

## Final Checklist □

- [ ] All code tested and working
- [ ] Report complete and formatted
- [ ] GitHub repository public and accessible
- [ ] Demo rehearsed successfully
- [ ] All group members understand the project
- [ ] World file (.sdf) ready to submit
- [ ] Launch files tested
- [ ] Performance data collected
- [ ] Screenshots/videos captured
- [ ] Ready for VIVA!

---

## Tips for Success

1. **Start Early:** Don't wait until the last minute
2. **Test Frequently:** Test each component before moving on
3. **Document As You Go:** Take notes and screenshots during development
4. **Backup Everything:** Use GitHub regularly
5. **Communicate:** Work together with your group
6. **Ask Questions:** Use ROS forums, Stack Overflow, and course resources
7. **Time Management:** SLAM mapping and DQL training take time!

## Time Estimates

- Setup & Installation: 2-3 hours
- Maze Creation: 2-3 hours
- SLAM Mapping: 2-4 hours
- Nav2 Navigation: 3-5 hours
- DQL Implementation: 10-20 hours (training)
- Testing & Data Collection: 4-6 hours
- Report Writing: 8-12 hours
- **Total: 30-50 hours**

Plan accordingly!

---

Good luck! 🚀

