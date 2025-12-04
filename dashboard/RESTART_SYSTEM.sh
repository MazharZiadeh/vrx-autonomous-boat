#!/bin/bash
# Complete System Restart Script
# Stops everything and restarts the full VRX system

set +e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m'

echo "=========================================="
echo "🔄 VRX System Restart"
echo "=========================================="
echo ""

# ============================================
# STEP 1: KILL ALL PROCESSES
# ============================================
echo -e "${YELLOW}1️⃣  Stopping all processes...${NC}"

# Kill Gazebo
echo "   Killing Gazebo..."
pkill -f "gz sim" 2>/dev/null || true
pkill -f "gazebo" 2>/dev/null || true
pkill -f "gz sim" 2>/dev/null || true

# Kill ROS nodes
echo "   Killing ROS nodes..."
pkill -f "ros2 launch" 2>/dev/null || true
pkill -f "rosbridge" 2>/dev/null || true
pkill -f "spawn.launch" 2>/dev/null || true

# Kill AIS processes
echo "   Killing AIS processes..."
pkill -f "ais_proxy.py" 2>/dev/null || true
pkill -f "ais_ros_bridge.py" 2>/dev/null || true
pkill -f "ais_gazebo_spawner.py" 2>/dev/null || true
pkill -f "spawned_boats_publisher.py" 2>/dev/null || true

# Kill keyboard control
echo "   Killing keyboard control..."
pkill -f "arrow_key_teleop.py" 2>/dev/null || true
pkill -f "twist_to_thrusters.py" 2>/dev/null || true

# Kill dashboard server
echo "   Killing dashboard server..."
pkill -f "python3 -m http.server" 2>/dev/null || true
pkill -f "dashboard" 2>/dev/null || true

# Kill any remaining Python ROS nodes
echo "   Killing remaining Python nodes..."
pkill -f "python3.*ros" 2>/dev/null || true

# Wait for processes to die
sleep 3

echo -e "${GREEN}✓ All processes stopped${NC}"
echo ""

# ============================================
# STEP 2: CLEAN UP PORTS
# ============================================
echo -e "${YELLOW}2️⃣  Cleaning up ports...${NC}"

# Free port 9090 (rosbridge)
if lsof -i :9090 > /dev/null 2>&1; then
    echo "   Freeing port 9090..."
    lsof -ti :9090 | xargs kill -9 2>/dev/null || true
fi

# Free port 9091 (AIS proxy)
if lsof -i :9091 > /dev/null 2>&1; then
    echo "   Freeing port 9091..."
    lsof -ti :9091 | xargs kill -9 2>/dev/null || true
fi

# Free port 8000 (dashboard)
if lsof -i :8000 > /dev/null 2>&1; then
    echo "   Freeing port 8000..."
    lsof -ti :8000 | xargs kill -9 2>/dev/null || true
fi

sleep 1
echo -e "${GREEN}✓ Ports cleaned${NC}"
echo ""

# ============================================
# STEP 3: SOURCE ROS ENVIRONMENT
# ============================================
echo -e "${YELLOW}3️⃣  Setting up ROS environment...${NC}"

source /opt/ros/jazzy/setup.bash
if [ -d "$HOME/vrx_ws" ]; then
    source $HOME/vrx_ws/install/setup.bash 2>/dev/null || true
    echo -e "${GREEN}✓ ROS environment sourced${NC}"
else
    echo -e "${YELLOW}⚠ vrx_ws not found, using system ROS only${NC}"
fi

echo ""

# ============================================
# STEP 4: START GAZEBO
# ============================================
echo -e "${YELLOW}4️⃣  Starting Gazebo...${NC}"

cd $HOME/vrx_ws

# Launch Gazebo in background
ros2 launch vrx_gz competition.launch.py world:=sydney_regatta > /tmp/gazebo.log 2>&1 &
GAZEBO_PID=$!

echo "   Gazebo PID: $GAZEBO_PID"
echo "   Waiting for Gazebo to initialize..."

# Wait for Gazebo to start
sleep 10

# Check if Gazebo is running
if ps -p $GAZEBO_PID > /dev/null; then
    echo -e "${GREEN}✓ Gazebo started${NC}"
else
    echo -e "${RED}✗ Gazebo failed to start - check /tmp/gazebo.log${NC}"
    exit 1
fi

echo ""

# ============================================
# STEP 5: SPAWN MAIN BOAT
# ============================================
echo -e "${YELLOW}5️⃣  Spawning main boat...${NC}"

# Wait a bit more for Gazebo to be ready
sleep 5

# Spawn main boat
ros2 launch vrx_gz spawn.launch.py > /tmp/spawn.log 2>&1 &
SPAWN_PID=$!

echo "   Spawn PID: $SPAWN_PID"
sleep 5

if ps -p $SPAWN_PID > /dev/null || grep -q "success" /tmp/spawn.log 2>/dev/null; then
    echo -e "${GREEN}✓ Main boat spawned${NC}"
else
    echo -e "${YELLOW}⚠ Spawn may have completed (check Gazebo)${NC}"
fi

echo ""

# ============================================
# STEP 6: START ROSBRIDGE
# ============================================
echo -e "${YELLOW}6️⃣  Starting rosbridge server...${NC}"

ros2 run rosbridge_server rosbridge_websocket --ros-args -p port:=9090 > /tmp/rosbridge.log 2>&1 &
ROSBRIDGE_PID=$!

echo "   rosbridge PID: $ROSBRIDGE_PID"
sleep 2

if ps -p $ROSBRIDGE_PID > /dev/null; then
    echo -e "${GREEN}✓ rosbridge started on port 9090${NC}"
else
    echo -e "${YELLOW}⚠ rosbridge may have failed - check /tmp/rosbridge.log${NC}"
fi

echo ""

# ============================================
# STEP 7: START DASHBOARD SERVER
# ============================================
echo -e "${YELLOW}7️⃣  Starting dashboard server...${NC}"

cd /home/mazhar/final_stand/vrx/dashboard
python3 -m http.server 8000 > /tmp/dashboard.log 2>&1 &
DASHBOARD_PID=$!

echo "   Dashboard PID: $DASHBOARD_PID"
sleep 1

if ps -p $DASHBOARD_PID > /dev/null; then
    echo -e "${GREEN}✓ Dashboard server started on port 8000${NC}"
else
    echo -e "${YELLOW}⚠ Dashboard server may have failed${NC}"
fi

echo ""

# ============================================
# STEP 8: SUMMARY
# ============================================
echo "=========================================="
echo -e "${GREEN}✅ System Restarted!${NC}"
echo "=========================================="
echo ""
echo "Process IDs:"
echo "  Gazebo:        $GAZEBO_PID"
echo "  Spawn:         $SPAWN_PID"
echo "  rosbridge:     $ROSBRIDGE_PID"
echo "  Dashboard:     $DASHBOARD_PID"
echo ""
echo "Access Points:"
echo "  Dashboard:     http://localhost:8000/dashboard.html"
echo "  rosbridge:     ws://localhost:9090"
echo ""
echo "Logs:"
echo "  Gazebo:        /tmp/gazebo.log"
echo "  Spawn:         /tmp/spawn.log"
echo "  rosbridge:     /tmp/rosbridge.log"
echo "  Dashboard:     /tmp/dashboard.log"
echo ""
echo "Next Steps:"
echo "  1. Open dashboard: http://localhost:8000/dashboard.html"
echo "  2. To control boat: cd dashboard && ./start_keyboard_control.sh"
echo "  3. To spawn NPC boats: cd dashboard && ./spawn_wamv_boats.sh"
echo ""
echo "To stop everything:"
echo "  pkill -f 'gz sim\|ros2 launch\|rosbridge\|http.server'"
echo ""

