#!/bin/bash
# Setup and start rosbridge_server for dashboard

echo "=========================================="
echo "🌐 ROSBridge Server Setup"
echo "=========================================="
echo ""

# Check if rosbridge is installed
if ! ros2 pkg list | grep -q rosbridge; then
    echo "Installing rosbridge_server..."
    sudo apt install ros-jazzy-rosbridge-suite -y
    echo "✅ rosbridge_server installed"
else
    echo "✅ rosbridge_server already installed"
fi

echo ""
echo "Starting rosbridge WebSocket server..."
echo "Server will be available at: ws://localhost:9090"
echo ""
echo "Press Ctrl+C to stop"
echo ""

cd /home/mazhar/vrx_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch rosbridge_server rosbridge_websocket_launch.xml

