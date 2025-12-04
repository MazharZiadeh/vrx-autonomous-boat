#!/bin/bash
# Start AIS Proxy and ROS Bridge

# Don't exit on error - we handle errors manually
set +e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo "=========================================="
echo "🚢 Starting AIS Integration System"
echo "=========================================="
echo ""

# Kill existing AIS processes
echo -e "${YELLOW}Cleaning up existing AIS processes...${NC}"
pkill -f "ais_proxy.py" 2>/dev/null || true
pkill -f "ais_ros_bridge.py" 2>/dev/null || true
pkill -f "ais_gazebo_spawner.py" 2>/dev/null || true
sleep 2

# Check if port 9091 is in use
if lsof -i :9091 > /dev/null 2>&1; then
    echo -e "${YELLOW}Port 9091 is in use, killing process...${NC}"
    lsof -ti :9091 | xargs kill -9 2>/dev/null || true
    sleep 1
fi

# Set API key
export AISSTREAM_API_KEY="70e4762d6c570a3204ec48ddac3c08e1649320a5"

# Check if Python dependencies are installed
echo -e "${YELLOW}Checking dependencies...${NC}"
python3 -c "import websockets" 2>/dev/null || {
    echo -e "${YELLOW}Installing websockets...${NC}"
    pip3 install websockets --user
}

# Step 1: Start AIS Proxy
echo -e "${YELLOW}1️⃣  Starting AIS Proxy...${NC}"
cd /home/mazhar/final_stand/vrx/dashboard
python3 ais_proxy.py > /tmp/ais_proxy.log 2>&1 &
AIS_PROXY_PID=$!
echo "   AIS Proxy PID: $AIS_PROXY_PID"
sleep 3

# Check if AIS proxy is running
if ps -p $AIS_PROXY_PID > /dev/null; then
    echo -e "${GREEN}✓ AIS Proxy running${NC}"
else
    echo -e "${RED}✗ AIS Proxy failed to start - check /tmp/ais_proxy.log${NC}"
    exit 1
fi

# Step 2: Start AIS ROS Bridge
echo -e "${YELLOW}2️⃣  Starting AIS ROS Bridge...${NC}"
source /opt/ros/jazzy/setup.bash
if [ -d "$HOME/vrx_ws" ]; then
    source $HOME/vrx_ws/install/setup.bash 2>/dev/null || true
fi

cd /home/mazhar/final_stand/vrx/dashboard
python3 ais_ros_bridge.py > /tmp/ais_ros_bridge.log 2>&1 &
AIS_ROS_PID=$!
echo "   AIS ROS Bridge PID: $AIS_ROS_PID"
sleep 2

if ps -p $AIS_ROS_PID > /dev/null; then
    echo -e "${GREEN}✓ AIS ROS Bridge running${NC}"
else
    echo -e "${RED}✗ AIS ROS Bridge failed to start - check /tmp/ais_ros_bridge.log${NC}"
fi

# Step 3: Start AIS Gazebo Spawner (optional - spawns vessels in Gazebo)
echo -e "${YELLOW}3️⃣  Starting AIS Gazebo Spawner...${NC}"
cd /home/mazhar/final_stand/vrx/dashboard
python3 ais_gazebo_spawner.py > /tmp/ais_gazebo_spawner.log 2>&1 &
AIS_GAZEBO_PID=$!
echo "   AIS Gazebo Spawner PID: $AIS_GAZEBO_PID"
sleep 2

if ps -p $AIS_GAZEBO_PID > /dev/null; then
    echo -e "${GREEN}✓ AIS Gazebo Spawner running${NC}"
else
    echo -e "${YELLOW}⚠ AIS Gazebo Spawner may have failed (Gazebo may not be running yet)${NC}"
fi

echo ""
echo "=========================================="
echo "✅ AIS System Started!"
echo "=========================================="
echo ""
echo "Process IDs:"
echo "  AIS Proxy:         $AIS_PROXY_PID"
echo "  AIS ROS Bridge:    $AIS_ROS_PID"
echo "  AIS Gazebo Spawner: $AIS_GAZEBO_PID"
echo ""
echo "Logs:"
echo "  AIS Proxy:         /tmp/ais_proxy.log"
echo "  AIS ROS Bridge:    /tmp/ais_ros_bridge.log"
echo "  AIS Gazebo Spawner: /tmp/ais_gazebo_spawner.log"
echo ""
echo "Dashboard: http://localhost:8000/dashboard.html"
echo ""
echo "To stop:"
echo "  kill $AIS_PROXY_PID $AIS_ROS_PID $AIS_GAZEBO_PID"
echo ""

