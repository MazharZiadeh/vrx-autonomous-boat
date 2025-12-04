#!/bin/bash

# Simple rosbridge launcher
echo "🌐 Starting rosbridge WebSocket Server..."
echo ""

# Source ROS
source /opt/ros/jazzy/setup.bash

# Launch rosbridge
echo "Launching rosbridge on port 9090..."
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

