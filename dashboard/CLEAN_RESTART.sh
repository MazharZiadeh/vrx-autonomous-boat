#!/bin/bash

# Clean restart - kills everything and starts fresh with ONE boat

echo "🧹 CLEANING UP - Killing Everything..."
echo ""

# Kill everything
pkill -f "gazebo\|gz sim\|rosbridge\|ros2 launch\|waypoint_navigator\|mission_manager\|http.server\|spawn" 2>/dev/null || true

echo "   Waiting 3 seconds..."
sleep 3

echo -e "✅ Everything killed"
echo ""
echo "=========================================="
echo "🚀 RESTARTING WITH ONE BOAT"
echo "=========================================="
echo ""

# Now use the full launch script
cd ~/final_stand/vrx/dashboard
./LAUNCH_FULL_SYSTEM.sh

