#!/bin/bash
# Quick test script for waypoint follower

echo "=========================================="
echo "🧭 Waypoint Follower Test"
echo "=========================================="
echo ""

# Get current GPS
echo "1. Getting current boat GPS position..."
cd /home/mazhar/vrx_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "Waiting for GPS fix..."
GPS_OUTPUT=$(timeout 5 ros2 topic echo /wamv/sensors/gps/gps/fix --once 2>/dev/null | grep -E "(latitude|longitude)" | head -2)

if [ -z "$GPS_OUTPUT" ]; then
    echo "❌ No GPS data found. Make sure VRX is running and boat is spawned!"
    exit 1
fi

echo "$GPS_OUTPUT"
echo ""
echo "2. To set a waypoint:"
echo "   Edit: /home/mazhar/vrx_ws/src/wamv_autonomy/wamv_autonomy/waypoint_follower.py"
echo "   Change lines 18-19:"
echo "     self.target_lat = <your_latitude>"
echo "     self.target_lon = <your_longitude>"
echo ""
echo "3. Then rebuild and run:"
echo "   cd /home/mazhar/vrx_ws"
echo "   colcon build --symlink-install --packages-select wamv_autonomy"
echo "   source install/setup.bash"
echo "   ros2 run wamv_autonomy waypoint_follower"
echo ""

