#!/bin/bash

echo "🚀 Starting NMEA Bridge for OpenCPN"
echo "===================================="
echo ""

# Source ROS
if ! command -v ros2 &> /dev/null; then
    echo "📦 Sourcing ROS 2..."
    source /opt/ros/jazzy/setup.bash
    if [ -f ~/vrx_ws/install/setup.bash ]; then
        source ~/vrx_ws/install/setup.bash
        echo "   ✅ ROS 2 sourced"
    fi
fi

# Check if already running
if pgrep -f "nmea_bridge.py" > /dev/null; then
    echo "⚠️  NMEA Bridge is already running (PID: $(pgrep -f nmea_bridge.py))"
    echo "   Kill it first with: pkill -f nmea_bridge.py"
    exit 1
fi

# Check if GPS topic exists
if ! timeout 2 ros2 topic list 2>/dev/null | grep -q "/wamv/sensors/gps"; then
    echo "⚠️  WARNING: GPS topic not found!"
    echo "   Make sure VRX is running and boat is spawned"
    echo "   Continuing anyway..."
fi

# Start NMEA bridge
echo "📡 Starting NMEA Bridge..."
cd ~/final_stand/vrx/opencpn
python3 nmea_bridge.py

