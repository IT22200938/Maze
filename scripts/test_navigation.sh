#!/bin/bash
# Test script for autonomous navigation
# Sends a series of goal poses to test the navigation system

echo "=== TurtleBot3 Navigation Test Script ==="
echo "Make sure the simulation is running!"
echo ""

# Wait for user confirmation
read -p "Press Enter to start sending goals..."

# Array of test goals (x, y coordinates)
GOALS=(
    "0.5 0.5"
    "2.5 0.5"
    "2.5 2.5"
    "0.5 2.5"
    "1.5 1.5"
)

# Send each goal with a delay
for i in "${!GOALS[@]}"; do
    COORDS=(${GOALS[$i]})
    X=${COORDS[0]}
    Y=${COORDS[1]}
    
    echo "[$((i+1))/${#GOALS[@]}] Sending goal to ($X, $Y)..."
    
    ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{
        header: {frame_id: 'map'},
        pose: {
            position: {x: $X, y: $Y, z: 0.0},
            orientation: {w: 1.0}
        }
    }"
    
    # Wait before sending next goal
    if [ $i -lt $((${#GOALS[@]} - 1)) ]; then
        echo "Waiting 30 seconds before next goal..."
        sleep 30
    fi
done

echo ""
echo "=== All goals sent! ==="
echo "Check navigation_logs/ for performance data"

