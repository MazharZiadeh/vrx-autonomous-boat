#!/bin/bash
# Start rosbridge server for web dashboard

cd /home/mazhar/vrx_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "=========================================="
echo "🌐 Starting rosbridge WebSocket Server"
echo "=========================================="
echo ""
echo "Server will be available at: ws://localhost:9090"
echo ""
echo "Press Ctrl+C to stop"
echo ""

ros2 launch rosbridge_server rosbridge_websocket_launch.xml

