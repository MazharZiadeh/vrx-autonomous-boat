#!/bin/bash

echo "🧪 Testing rosbridge Connection"
echo "================================"
echo ""

# Source ROS
source /opt/ros/jazzy/setup.bash 2>/dev/null

# Kill existing
pkill -f rosbridge 2>/dev/null
sleep 2

echo "1. Starting rosbridge..."
ros2 launch rosbridge_server rosbridge_websocket_launch.xml > /tmp/rosbridge_test.log 2>&1 &
ROSBRIDGE_PID=$!
echo "   PID: $ROSBRIDGE_PID"

echo ""
echo "2. Waiting 5 seconds for startup..."
sleep 5

echo ""
echo "3. Checking if process is running..."
if ps -p $ROSBRIDGE_PID > /dev/null 2>&1; then
    echo "   ✅ Process is running"
else
    echo "   ❌ Process died!"
    echo "   Logs:"
    tail -20 /tmp/rosbridge_test.log
    exit 1
fi

echo ""
echo "4. Checking if port 9090 is listening..."
if lsof -i :9090 > /dev/null 2>&1; then
    echo "   ✅ Port 9090 is listening"
    lsof -i :9090
else
    echo "   ❌ Port 9090 is NOT listening"
    echo "   This is the problem!"
    echo ""
    echo "   Checking logs for errors:"
    tail -30 /tmp/rosbridge_test.log | grep -i error || tail -30 /tmp/rosbridge_test.log
    exit 1
fi

echo ""
echo "5. Testing WebSocket connection..."
timeout 2 curl -i -N -H "Connection: Upgrade" -H "Upgrade: websocket" -H "Sec-WebSocket-Version: 13" -H "Sec-WebSocket-Key: test" http://localhost:9090 2>&1 | head -5

echo ""
echo "✅ rosbridge appears to be working!"
echo ""
echo "To keep it running, don't close this terminal."
echo "Or run in background:"
echo "  nohup ros2 launch rosbridge_server rosbridge_websocket_launch.xml > /tmp/rosbridge.log 2>&1 &"

