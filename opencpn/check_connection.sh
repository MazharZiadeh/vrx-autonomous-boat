#!/bin/bash

echo "🔍 Checking OpenCPN Connection Status"
echo "======================================"
echo ""

# Check if ROS is sourced
if ! command -v ros2 &> /dev/null; then
    echo "⚠️  ROS 2 not found. Sourcing..."
    source /opt/ros/jazzy/setup.bash
    if [ -f ~/vrx_ws/install/setup.bash ]; then
        source ~/vrx_ws/install/setup.bash
    fi
fi

echo "1️⃣ Checking VRX/Gazebo..."
if pgrep -f "gazebo\|gz sim" > /dev/null; then
    echo "   ✅ Gazebo is running"
else
    echo "   ❌ Gazebo NOT running - VRX needs to be launched!"
    echo "      Run: ros2 launch vrx_gz competition.launch.py world:=sydney_regatta"
fi

echo ""
echo "2️⃣ Checking if boat is spawned..."
if ros2 topic list 2>/dev/null | grep -q "/wamv/sensors/gps"; then
    echo "   ✅ Boat is spawned (GPS topic exists)"
else
    echo "   ❌ Boat NOT spawned - no GPS topics found"
    echo "      Run: ros2 launch vrx_gz spawn.launch.py"
fi

echo ""
echo "3️⃣ Checking GPS data..."
GPS_DATA=$(timeout 2 ros2 topic echo /wamv/sensors/gps/gps/fix --once 2>&1 | grep -E "latitude|longitude" | head -2)
if [ ! -z "$GPS_DATA" ]; then
    echo "   ✅ GPS is publishing data:"
    echo "      $GPS_DATA"
else
    echo "   ❌ GPS NOT publishing - boat might not be spawned or VRX not running"
fi

echo ""
echo "4️⃣ Checking NMEA Bridge..."
if pgrep -f "nmea_bridge.py" > /dev/null; then
    echo "   ✅ NMEA Bridge is running (PID: $(pgrep -f nmea_bridge.py))"
else
    echo "   ❌ NMEA Bridge NOT running"
    echo "      Run: cd ~/final_stand/vrx/opencpn && python3 nmea_bridge.py"
fi

echo ""
echo "5️⃣ Checking UDP port 10110..."
if lsof -i :10110 2>/dev/null | grep -q LISTEN; then
    echo "   ✅ Port 10110 is in use (NMEA bridge should be sending)"
else
    echo "   ⚠️  Port 10110 not listening (NMEA bridge uses UDP, might not show as LISTEN)"
fi

echo ""
echo "6️⃣ Testing NMEA data flow..."
if pgrep -f "nmea_bridge.py" > /dev/null && [ ! -z "$GPS_DATA" ]; then
    echo "   ✅ Both NMEA bridge and GPS are active - data should be flowing!"
    echo "   💡 Check OpenCPN NMEA Debug Window - you should see sentences"
else
    echo "   ❌ Missing components - fix above issues first"
fi

echo ""
echo "=========================================="
echo "📋 QUICK FIX CHECKLIST:"
echo ""
echo "If GPS not publishing:"
echo "  1. Launch VRX: ros2 launch vrx_gz competition.launch.py world:=sydney_regatta"
echo "  2. Wait 20 seconds"
echo "  3. Spawn boat: ros2 launch vrx_gz spawn.launch.py"
echo ""
echo "If NMEA bridge not running:"
echo "  cd ~/final_stand/vrx/opencpn"
echo "  source /opt/ros/jazzy/setup.bash"
echo "  source ~/vrx_ws/install/setup.bash"
echo "  python3 nmea_bridge.py"
echo ""
echo "If OpenCPN shows no data:"
echo "  1. Verify connection settings (127.0.0.1:10110, UDP, Input)"
echo "  2. Enable NMEA Debug Window in OpenCPN"
echo "  3. Check NMEA bridge logs for errors"
echo ""

