#!/bin/bash

echo "🔧 Fixing OpenCPN Connection - Step by Step"
echo "==========================================="
echo ""

source /opt/ros/jazzy/setup.bash 2>/dev/null
if [ -f ~/vrx_ws/install/setup.bash ]; then
    source ~/vrx_ws/install/setup.bash
fi

echo "Step 1: Kill old NMEA Bridge (if running)..."
pkill -f nmea_bridge.py 2>/dev/null
sleep 1
echo "   ✅ Done"
echo ""

echo "Step 2: Wait for GPS to start publishing..."
echo "   Checking GPS topic (this may take 10-20 seconds)..."
for i in {1..20}; do
    GPS_DATA=$(timeout 2 ros2 topic echo /wamv/sensors/gps/gps/fix --once 2>&1 | grep -E "latitude|longitude" | head -2)
    if [ ! -z "$GPS_DATA" ]; then
        echo "   ✅ GPS is now publishing!"
        echo "      $GPS_DATA"
        break
    else
        echo "   ⏳ Waiting... ($i/20)"
        sleep 1
    fi
done

if [ -z "$GPS_DATA" ]; then
    echo "   ⚠️  GPS still not publishing after 20 seconds"
    echo "   💡 Make sure VRX is fully running and boat is spawned"
    echo "   💡 Check Gazebo window - boat should be visible"
fi

echo ""
echo "Step 3: Start NMEA Bridge..."
cd ~/final_stand/vrx/opencpn
python3 nmea_bridge.py &
NMEA_PID=$!
sleep 2

if pgrep -f nmea_bridge.py > /dev/null; then
    echo "   ✅ NMEA Bridge started (PID: $NMEA_PID)"
    echo "   💡 Keep this terminal open - NMEA bridge is running in background"
else
    echo "   ❌ Failed to start NMEA Bridge"
    echo "   💡 Try running manually: python3 ~/final_stand/vrx/opencpn/nmea_bridge.py"
fi

echo ""
echo "Step 4: Verify everything is working..."
sleep 2

GPS_CHECK=$(timeout 2 ros2 topic echo /wamv/sensors/gps/gps/fix --once 2>&1 | grep -E "latitude|longitude" | head -2)
NMEA_CHECK=$(pgrep -f nmea_bridge.py)

echo ""
echo "=========================================="
echo "📊 FINAL STATUS:"
echo ""

if [ ! -z "$GPS_CHECK" ] && [ ! -z "$NMEA_CHECK" ]; then
    echo "   ✅ GPS is publishing"
    echo "   ✅ NMEA Bridge is running"
    echo ""
    echo "   🎉 Everything should be working!"
    echo ""
    echo "   💡 Check OpenCPN:"
    echo "      - NMEA Debug Window should show incoming sentences"
    echo "      - Boat icon should appear on map"
    echo "      - Position should update in real-time"
else
    echo "   ⚠️  Some components still not working:"
    [ -z "$GPS_CHECK" ] && echo "      ❌ GPS not publishing"
    [ -z "$NMEA_CHECK" ] && echo "      ❌ NMEA Bridge not running"
    echo ""
    echo "   💡 Troubleshooting:"
    echo "      - If GPS not publishing: Wait longer or restart VRX"
    echo "      - If NMEA Bridge not running: Check for errors in terminal"
fi

echo ""
echo "=========================================="
echo ""
echo "📋 To check status later:"
echo "   cd ~/final_stand/vrx/opencpn"
echo "   ./check_connection.sh"
echo ""

