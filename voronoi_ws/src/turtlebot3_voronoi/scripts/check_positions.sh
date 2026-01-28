#!/bin/bash
# Compare actual Gazebo model positions with controller world_pose.
# Run while the simulation is active.

echo "=== Gazebo Ground Truth (gz model -p) ==="
for i in 0 1 2 3 4; do
    echo -n "tb3_${i}: "
    gz model -m "tb3_${i}" -p 2>/dev/null | head -3
    echo "---"
done

echo ""
echo "=== Controller world_pose (ROS topics) ==="
for i in 0 1 2 3 4; do
    echo -n "tb3_${i}: "
    timeout 2 ros2 topic echo "/tb3_${i}/world_pose" --once 2>/dev/null | grep -A2 "position:" | head -3
    echo "---"
done
