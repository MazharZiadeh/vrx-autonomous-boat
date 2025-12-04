#!/bin/bash

# Demo Mission Script
# Launches autonomy and sends a test mission

echo "🚤 Starting Autonomous Mission Demo..."
echo ""

cd ~/vrx_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Check if VRX is running
if ! ros2 topic list 2>/dev/null | grep -q "/wamv/sensors/gps"; then
    echo "❌ VRX not running or boat not spawned!"
    echo "   Please launch VRX first:"
    echo "   ros2 launch vrx_gz competition.launch.py world:=sydney_regatta"
    exit 1
fi

# Launch autonomy node
echo "1️⃣  Launching autonomy node..."
ros2 launch wamv_autonomy autonomy.launch.py \
    heading_tolerance:=15.0 \
    waypoint_threshold:=10.0 \
    thrust_power:=50.0 > /tmp/autonomy_demo.log 2>&1 &
AUTONOMY_PID=$!
echo "   Autonomy PID: $AUTONOMY_PID"
sleep 3

# Get current position
echo ""
echo "2️⃣  Getting current boat position..."
sleep 2
CURRENT_POS=$(timeout 3 ros2 topic echo /wamv/sensors/gps/gps/fix --once 2>/dev/null)
if [ -z "$CURRENT_POS" ]; then
    echo "   ⚠️  Could not get position, using default"
    LAT="-33.7228"
    LON="150.6740"
else
    LAT=$(echo "$CURRENT_POS" | grep "latitude:" | awk '{print $2}')
    LON=$(echo "$CURRENT_POS" | grep "longitude:" | awk '{print $2}')
fi
echo "   Current: Lat=$LAT, Lon=$LON"

# Send Mission 1 (Simple Square)
echo ""
echo "3️⃣  Sending Mission 1: Simple Square (4 waypoints)..."
sleep 1

# Calculate waypoints (small square pattern)
WP1_LAT=$(echo "$LAT" | awk '{print $1}')
WP1_LON=$(echo "$LON" | awk '{print $1}')
WP2_LAT="$WP1_LAT"
WP2_LON=$(echo "$LON + 0.0005" | bc -l 2>/dev/null || echo "$LON")
WP3_LAT=$(echo "$LAT - 0.0005" | bc -l 2>/dev/null || echo "$LAT")
WP3_LON="$WP2_LON"
WP4_LAT="$WP3_LAT"
WP4_LON="$WP1_LON"

ros2 topic pub /wamv/mission/waypoints geometry_msgs/msg/PoseArray "
header:
  frame_id: 'map'
poses:
- position: {x: $WP1_LON, y: $WP1_LAT, z: 0.0}
- position: {x: $WP2_LON, y: $WP2_LAT, z: 0.0}
- position: {x: $WP3_LON, y: $WP3_LAT, z: 0.0}
- position: {x: $WP4_LON, y: $WP4_LAT, z: 0.0}
" --once

echo "   ✅ Mission sent!"
echo ""
echo "=========================================="
echo "✅ DEMO STARTED!"
echo "=========================================="
echo ""
echo "Watch the dashboard and Gazebo:"
echo "  - Waypoints appear on map (amber circles)"
echo "  - Boat starts navigating"
echo "  - Current waypoint turns green"
echo "  - Completed waypoints turn gray"
echo ""
echo "Monitor status:"
echo "  ros2 topic echo /wamv/mission/status"
echo "  ros2 topic echo /wamv/mission/current_waypoint"
echo ""
echo "Autonomy PID: $AUTONOMY_PID"
echo "To stop: pkill -f waypoint_navigator"
echo ""
echo "Dashboard: http://localhost:8000/dashboard.html"
echo ""

