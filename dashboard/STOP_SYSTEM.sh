#!/bin/bash
# Stop All VRX System Processes

set +e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo "=========================================="
echo "🛑 Stopping VRX System"
echo "=========================================="
echo ""

# Kill Gazebo
echo -e "${YELLOW}Stopping Gazebo...${NC}"
pkill -f "gz sim" 2>/dev/null || true
pkill -f "gazebo" 2>/dev/null || true
sleep 2

# Kill ROS nodes
echo -e "${YELLOW}Stopping ROS nodes...${NC}"
pkill -f "ros2 launch" 2>/dev/null || true
pkill -f "rosbridge" 2>/dev/null || true
pkill -f "spawn.launch" 2>/dev/null || true
sleep 2

# Kill AIS processes
echo -e "${YELLOW}Stopping AIS processes...${NC}"
pkill -f "ais_proxy.py" 2>/dev/null || true
pkill -f "ais_ros_bridge.py" 2>/dev/null || true
pkill -f "ais_gazebo_spawner.py" 2>/dev/null || true
pkill -f "spawned_boats_publisher.py" 2>/dev/null || true

# Kill keyboard control
echo -e "${YELLOW}Stopping keyboard control...${NC}"
pkill -f "arrow_key_teleop.py" 2>/dev/null || true
pkill -f "twist_to_thrusters.py" 2>/dev/null || true

# Kill dashboard server
echo -e "${YELLOW}Stopping dashboard server...${NC}"
pkill -f "python3 -m http.server" 2>/dev/null || true

# Kill any remaining Python ROS nodes
pkill -f "python3.*ros" 2>/dev/null || true

# Free ports
echo -e "${YELLOW}Freeing ports...${NC}"
lsof -ti :9090 | xargs kill -9 2>/dev/null || true
lsof -ti :9091 | xargs kill -9 2>/dev/null || true
lsof -ti :8000 | xargs kill -9 2>/dev/null || true

sleep 1

echo ""
echo -e "${GREEN}✅ All processes stopped${NC}"
echo ""

