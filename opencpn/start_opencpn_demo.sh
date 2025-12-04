#!/bin/bash

echo "🗺️  Starting OpenCPN Integration Demo"
echo "======================================"

# Check if ROS is sourced
if ! command -v ros2 &> /dev/null; then
    echo "⚠️  ROS 2 not found. Sourcing..."
    source /opt/ros/jazzy/setup.bash
    if [ -f ~/vrx_ws/install/setup.bash ]; then
        source ~/vrx_ws/install/setup.bash
    fi
fi

# Start NMEA bridge
echo "📡 Starting NMEA Bridge..."
cd ~/final_stand/vrx/opencpn
python3 nmea_bridge.py &
NMEA_PID=$!
sleep 2

# Check if OpenCPN is installed
if ! command -v opencpn &> /dev/null; then
    echo "⚠️  OpenCPN not found. Install with:"
    echo "   sudo apt install opencpn"
    echo ""
    echo "   Or download from: https://opencpn.org/OpenCPN/info/downloads.html"
    echo ""
    echo "   Continuing without OpenCPN..."
else
    # Start OpenCPN
    echo "🗺️  Starting OpenCPN..."
    opencpn &
    OPENCPN_PID=$!
    sleep 3
fi

echo ""
echo "✅ NMEA Bridge running (PID: $NMEA_PID)"
if [ ! -z "$OPENCPN_PID" ]; then
    echo "✅ OpenCPN running (PID: $OPENCPN_PID)"
fi
echo ""
echo "📋 Next steps:"
echo "   1. In OpenCPN, configure connection:"
echo "      - Options → Connections → Add Connection"
echo "      - Network (UDP), Address: 127.0.0.1, Port: 10110"
echo "      - Protocol: NMEA 0183, Direction: Input"
echo ""
echo "   2. Draw a route on the map (right-click → New Route)"
echo ""
echo "   3. Export route as GPX file"
echo ""
echo "   4. Load route into waypoint navigator:"
echo "      python3 ~/final_stand/vrx/opencpn/route_bridge.py <gpx_file>"
echo ""
echo "   5. Start waypoint navigator:"
echo "      ros2 launch wamv_autonomy autonomy.launch.py"
echo ""
echo "Press Ctrl+C to stop all"

# Wait for Ctrl+C
trap "kill $NMEA_PID 2>/dev/null; kill $OPENCPN_PID 2>/dev/null; exit" INT TERM
wait

