#!/bin/bash
# Manual start script for Random Boats Spawner
# Run this in a separate terminal after Gazebo is running

echo "🚢 Starting Random Boats Spawner..."
echo ""

# Source ROS
source /opt/ros/jazzy/setup.bash
if [ -d "$HOME/vrx_ws" ]; then
    source $HOME/vrx_ws/install/setup.bash 2>/dev/null || true
fi

cd /home/mazhar/final_stand/vrx/dashboard

echo "✅ ROS environment sourced"
echo "🚤 Starting random boats spawner..."
echo "   (Press Ctrl+C to stop)"
echo ""

python3 random_boats_spawner.py

