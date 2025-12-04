#!/bin/bash

# Correct way to launch fixed rosbridge

echo "🌐 Starting rosbridge with FIXED launch file..."
echo ""

# Source ROS
source /opt/ros/jazzy/setup.bash

# Kill existing
pkill -f rosbridge 2>/dev/null
sleep 1

# Launch with full path
cd ~/final_stand/vrx/dashboard
ros2 launch $(pwd)/rosbridge_fixed.launch.py

