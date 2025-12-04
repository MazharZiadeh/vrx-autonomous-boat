#!/bin/bash

echo "🔍 Debugging NMEA Bridge Connection"
echo "===================================="
echo ""

source /opt/ros/jazzy/setup.bash 2>/dev/null
if [ -f ~/vrx_ws/install/setup.bash ]; then
    source ~/vrx_ws/install/setup.bash
fi

echo "1️⃣ Checking GPS topic..."
GPS_TOPIC="/wamv/sensors/gps/gps/fix"
if ros2 topic list 2>/dev/null | grep -q "$GPS_TOPIC"; then
    echo "   ✅ GPS topic exists: $GPS_TOPIC"
    
    echo ""
    echo "2️⃣ Checking if GPS is publishing..."
    GPS_DATA=$(timeout 3 ros2 topic echo $GPS_TOPIC --once 2>&1 | grep -E "latitude|longitude" | head -2)
    if [ ! -z "$GPS_DATA" ]; then
        echo "   ✅ GPS is publishing:"
        echo "      $GPS_DATA"
    else
        echo "   ❌ GPS NOT publishing - topic exists but no data"
        echo "   💡 Wait a few seconds for VRX to fully initialize"
    fi
else
    echo "   ❌ GPS topic NOT found: $GPS_TOPIC"
    echo "   📋 Available GPS topics:"
    ros2 topic list 2>/dev/null | grep -i gps | head -5
fi

echo ""
echo "3️⃣ Checking NMEA Bridge process..."
if pgrep -f "nmea_bridge.py" > /dev/null; then
    NMEA_PID=$(pgrep -f nmea_bridge.py)
    echo "   ✅ NMEA Bridge is running (PID: $NMEA_PID)"
    
    echo ""
    echo "4️⃣ Checking NMEA Bridge logs (last 10 lines)..."
    # Try to get recent logs if possible
    echo "   💡 Check the terminal where nmea_bridge.py is running"
    echo "   💡 Look for: '📡 Sent NMEA: Lat=...' messages"
else
    echo "   ❌ NMEA Bridge NOT running"
    echo "   💡 Start it with: python3 ~/final_stand/vrx/opencpn/nmea_bridge.py"
fi

echo ""
echo "5️⃣ Testing UDP port 10110..."
if lsof -i :10110 2>/dev/null | grep -q LISTEN; then
    echo "   ✅ Port 10110 is in use"
else
    echo "   ⚠️  Port 10110 not showing as LISTEN (UDP might not show this)"
fi

echo ""
echo "6️⃣ Manual NMEA test..."
echo "   Sending test NMEA sentence to OpenCPN..."
echo '$GPGGA,123456.00,3354.1234,S,15040.5678,E,1,08,1.0,0.5,M,0.0,M,,*XX' | nc -u -w 1 127.0.0.1 10110 2>&1
if [ $? -eq 0 ]; then
    echo "   ✅ Test sentence sent"
    echo "   💡 Check OpenCPN NMEA Debug Window - you should see the test sentence"
else
    echo "   ⚠️  Could not send test (might be normal)"
fi

echo ""
echo "=========================================="
echo "📋 TROUBLESHOOTING STEPS:"
echo ""
echo "If GPS not publishing:"
echo "   1. Wait 10-20 seconds after VRX starts"
echo "   2. Check Gazebo window - boat should be visible"
echo "   3. Try: ros2 topic echo /wamv/sensors/gps/gps/fix"
echo ""
echo "If NMEA Bridge not receiving data:"
echo "   1. Check NMEA bridge terminal for errors"
echo "   2. Verify GPS topic name matches in nmea_bridge.py"
echo "   3. Restart NMEA bridge: pkill -f nmea_bridge.py && python3 nmea_bridge.py"
echo ""
echo "If OpenCPN still shows no data:"
echo "   1. Verify connection: UDP, 127.0.0.1:10110, Input"
echo "   2. Enable NMEA Debug Window in OpenCPN"
echo "   3. Restart OpenCPN after connection setup"
echo "   4. Check firewall: sudo ufw allow 10110/udp"
echo ""

