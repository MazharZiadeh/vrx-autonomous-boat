#!/bin/bash

# Test if NMEA sentences can be sent to OpenCPN

echo "🧪 Testing NMEA Data Flow"
echo "=========================="
echo ""

# Test UDP port
echo "1️⃣ Testing UDP port 10110..."
echo '$GPGGA,123456.00,3354.1234,S,15040.5678,E,1,08,1.0,0.5,M,0.0,M,,*XX' | nc -u -w 1 127.0.0.1 10110 2>&1

if [ $? -eq 0 ]; then
    echo "   ✅ UDP port is accessible"
    echo "   💡 Check OpenCPN NMEA Debug Window - you should see the test sentence"
else
    echo "   ⚠️  Could not send to port (might be normal if nothing is listening)"
fi

echo ""
echo "2️⃣ Checking if NMEA bridge is receiving GPS data..."

source /opt/ros/jazzy/setup.bash 2>/dev/null
if [ -f ~/vrx_ws/install/setup.bash ]; then
    source ~/vrx_ws/install/setup.bash
fi

GPS_CHECK=$(timeout 2 ros2 topic echo /wamv/sensors/gps/gps/fix --once 2>&1 | grep -E "latitude|longitude" | head -2)

if [ ! -z "$GPS_CHECK" ]; then
    echo "   ✅ GPS is publishing:"
    echo "      $GPS_CHECK"
else
    echo "   ❌ GPS not publishing - VRX might not be running"
fi

echo ""
echo "3️⃣ If NMEA bridge is running, check its logs:"
echo "   Look for: '📡 Sent NMEA: Lat=...' messages"
echo ""

echo "✅ Test complete!"
echo ""
echo "💡 If OpenCPN still shows no data:"
echo "   1. Verify connection settings (UDP, 127.0.0.1:10110, Input)"
echo "   2. Enable NMEA Debug Window in OpenCPN"
echo "   3. Restart OpenCPN after connection setup"

