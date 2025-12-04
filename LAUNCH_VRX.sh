#!/bin/bash
# Launch VRX Simulation - Run this FIRST in Terminal 1

cd /home/mazhar/vrx_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "=========================================="
echo "🚤 Launching VRX Simulation..."
echo "=========================================="
echo ""
echo "This will:"
echo "  - Start Gazebo with ocean world"
echo "  - Spawn WAM-V boat"
echo "  - Bridge Gazebo topics to ROS 2"
echo ""
echo "⏳ Please wait 10-15 seconds for initialization..."
echo ""

ros2 launch vrx_gz vrx_environment.launch.py world:=sydney_regatta sim_mode:=full

