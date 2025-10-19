#!/bin/bash

# Build and Test Script for TurtleBot3 Maze Navigation
# This script builds the package and runs basic checks

set -e  # Exit on error

echo "=========================================="
echo "TurtleBot3 Maze Navigation - Build & Test"
echo "=========================================="
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check if we're in the right directory
if [ ! -f "package.xml" ]; then
    echo -e "${RED}Error: package.xml not found!${NC}"
    echo "Please run this script from the package root directory"
    exit 1
fi

# Get workspace path
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WS_DIR="$HOME/turtlebot3_ws"

echo "Step 1: Checking ROS 2 environment..."
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${YELLOW}ROS 2 not sourced. Sourcing...${NC}"
    source /opt/ros/jazzy/setup.bash
fi
echo -e "${GREEN}✓ ROS 2 $ROS_DISTRO environment active${NC}"
echo ""

echo "Step 2: Setting environment variables..."
export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=30
echo -e "${GREEN}✓ TURTLEBOT3_MODEL=$TURTLEBOT3_MODEL${NC}"
echo -e "${GREEN}✓ ROS_DOMAIN_ID=$ROS_DOMAIN_ID${NC}"
echo ""

echo "Step 3: Making scripts executable..."
chmod +x scripts/*.py
chmod +x scripts/*.bash
echo -e "${GREEN}✓ Scripts are now executable${NC}"
echo ""

echo "Step 4: Navigating to workspace..."
cd "$WS_DIR"
echo -e "${GREEN}✓ In workspace: $WS_DIR${NC}"
echo ""

echo "Step 5: Building package..."
echo "Running: colcon build --packages-select turtlebot3_maze_navigation"
if colcon build --packages-select turtlebot3_maze_navigation; then
    echo -e "${GREEN}✓ Build successful!${NC}"
else
    echo -e "${RED}✗ Build failed!${NC}"
    exit 1
fi
echo ""

echo "Step 6: Sourcing workspace..."
source install/setup.bash
echo -e "${GREEN}✓ Workspace sourced${NC}"
echo ""

echo "Step 7: Verifying package installation..."
if ros2 pkg list | grep -q "turtlebot3_maze_navigation"; then
    echo -e "${GREEN}✓ Package found in ROS 2${NC}"
else
    echo -e "${RED}✗ Package not found!${NC}"
    exit 1
fi
echo ""

echo "Step 8: Running setup verification..."
cd "$SCRIPT_DIR"
if python3 scripts/test_setup.py; then
    echo -e "${GREEN}✓ Setup verification passed!${NC}"
else
    echo -e "${YELLOW}⚠ Some checks failed. Review output above.${NC}"
fi
echo ""

echo "=========================================="
echo -e "${GREEN}BUILD COMPLETE!${NC}"
echo "=========================================="
echo ""
echo "Your package is ready to use!"
echo ""
echo "Quick Start Commands:"
echo ""
echo "1. Launch Gazebo with maze:"
echo "   ros2 launch turtlebot3_maze_navigation gazebo_maze.launch.py"
echo ""
echo "2. Start SLAM mapping:"
echo "   ros2 launch turtlebot3_maze_navigation cartographer.launch.py"
echo ""
echo "3. Start autonomous navigation:"
echo "   ros2 launch turtlebot3_maze_navigation navigation.launch.py"
echo ""
echo "For detailed instructions, see:"
echo "  - README.md (comprehensive guide)"
echo "  - QUICK_START.md (condensed guide)"
echo ""
echo "=========================================="

