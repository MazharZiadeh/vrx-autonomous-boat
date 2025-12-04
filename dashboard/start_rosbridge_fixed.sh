#!/bin/bash

# Fixed rosbridge launcher (fixes parameter type error)

echo "🌐 Starting rosbridge with FIXED launch file..."
echo ""

# Source ROS
source /opt/ros/jazzy/setup.bash

# Kill existing
pkill -f rosbridge 2>/dev/null
sleep 1

# Launch with fixed parameters
cd ~/final_stand/vrx/dashboard
ros2 launch dashboard/rosbridge_fixed.launch.py

