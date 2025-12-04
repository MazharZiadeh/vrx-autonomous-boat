#!/bin/bash

# ============================================
# MASTER LAUNCH SCRIPT - VRX + Dashboard
# ============================================
# This script launches everything needed for the demo
# Run this and everything should work!

set -e  # Exit on error

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo "=========================================="
echo "🚀 LAUNCHING VRX AUTONOMOUS BOAT SYSTEM"
echo "=========================================="
echo ""

# Check if ROS is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${YELLOW}⚠ Sourcing ROS 2...${NC}"
    if [ -f "/opt/ros/jazzy/setup.bash" ]; then
        source /opt/ros/jazzy/setup.bash
    elif [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
    else
        echo -e "${RED}✗ ROS 2 not found!${NC}"
        exit 1
    fi
fi

# Check VRX workspace
if [ ! -d "$HOME/vrx_ws" ]; then
    echo -e "${RED}✗ VRX workspace not found at $HOME/vrx_ws${NC}"
    exit 1
fi

cd $HOME/vrx_ws
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
else
    echo -e "${RED}✗ VRX workspace not built! Run: colcon build${NC}"
    exit 1
fi

# Check rosbridge
if ! ros2 pkg list | grep -q "rosbridge"; then
    echo -e "${YELLOW}⚠ rosbridge not installed. Installing...${NC}"
    sudo apt install -y ros-${ROS_DISTRO}-rosbridge-suite
fi

# Kill any existing processes
echo -e "${YELLOW}🧹 Cleaning up existing processes...${NC}"
pkill -f "gazebo\|gz sim\|rosbridge\|ros2 launch" || true
sleep 2

# Create log directory
LOG_DIR="$HOME/final_stand/vrx/dashboard/logs"
mkdir -p "$LOG_DIR"

# ============================================
# STEP 1: Launch VRX Environment
# ============================================
echo ""
echo -e "${GREEN}1️⃣  Launching VRX Environment...${NC}"
cd $HOME/vrx_ws
ros2 launch vrx_gz competition.launch.py world:=sydney_regatta > "$LOG_DIR/vrx.log" 2>&1 &
VRX_PID=$!
echo "   VRX PID: $VRX_PID"
echo "   Waiting 20 seconds for world to load..."
sleep 20

# Check if VRX is still running
if ! ps -p $VRX_PID > /dev/null; then
    echo -e "${RED}✗ VRX failed to start! Check $LOG_DIR/vrx.log${NC}"
    exit 1
fi

echo -e "${GREEN}✓ VRX environment loaded${NC}"

# ============================================
# STEP 2: Spawn WAM-V Boat
# ============================================
echo ""
echo -e "${GREEN}2️⃣  Spawning WAM-V Boat...${NC}"
ros2 launch vrx_gz spawn.launch.py > "$LOG_DIR/spawn.log" 2>&1 &
SPAWN_PID=$!
echo "   Spawn PID: $SPAWN_PID"
echo "   Waiting 10 seconds for boat to spawn..."
sleep 10

echo -e "${GREEN}✓ Boat spawned${NC}"

# ============================================
# STEP 3: Launch rosbridge (FIXED VERSION)
# ============================================
echo ""
echo -e "${GREEN}3️⃣  Launching rosbridge WebSocket server (FIXED)...${NC}"
cd $HOME/final_stand/vrx/dashboard
ros2 launch $(pwd)/rosbridge_fixed.launch.py > "$LOG_DIR/rosbridge.log" 2>&1 &
ROSBRIDGE_PID=$!
echo "   rosbridge PID: $ROSBRIDGE_PID"
echo "   Waiting 5 seconds for rosbridge to start..."
sleep 5

# Check if port 9090 is in use
if lsof -i :9090 > /dev/null 2>&1; then
    echo -e "${GREEN}✓ rosbridge running on port 9090${NC}"
else
    echo -e "${YELLOW}⚠ Port 9090 not in use - rosbridge may have failed${NC}"
    echo "   Check $LOG_DIR/rosbridge.log"
fi

# ============================================
# STEP 4: Launch Dashboard Server
# ============================================
echo ""
echo -e "${GREEN}4️⃣  Starting Dashboard Server...${NC}"
cd $HOME/final_stand/vrx/dashboard
python3 -m http.server 8000 > "$LOG_DIR/dashboard.log" 2>&1 &
DASHBOARD_PID=$!
echo "   Dashboard PID: $DASHBOARD_PID"
sleep 2

# Check if port 8000 is in use
if lsof -i :8000 > /dev/null 2>&1; then
    echo -e "${GREEN}✓ Dashboard server running on port 8000${NC}"
else
    echo -e "${YELLOW}⚠ Port 8000 not in use - dashboard server may have failed${NC}"
fi

# ============================================
# STEP 5: Open Browser
# ============================================
echo ""
echo -e "${GREEN}5️⃣  Opening Dashboard in Browser...${NC}"
sleep 2

# Try to open browser (works on most Linux systems)
if command -v xdg-open &> /dev/null; then
    xdg-open "http://localhost:8000/dashboard.html" 2>/dev/null &
elif command -v gnome-open &> /dev/null; then
    gnome-open "http://localhost:8000/dashboard.html" 2>/dev/null &
else
    echo -e "${YELLOW}⚠ Could not auto-open browser${NC}"
    echo "   Please open: http://localhost:8000/dashboard.html"
fi

# ============================================
# SUMMARY
# ============================================
echo ""
echo "=========================================="
echo "✅ SYSTEM LAUNCHED!"
echo "=========================================="
echo ""
echo "Process IDs (save these to kill later):"
echo "  VRX:        $VRX_PID"
echo "  Spawn:      $SPAWN_PID"
echo "  rosbridge:  $ROSBRIDGE_PID"
echo "  Dashboard:  $DASHBOARD_PID"
echo ""
echo "Dashboard URL: http://localhost:8000/dashboard.html"
echo ""
echo "Logs: $LOG_DIR/"
echo ""
echo "To stop everything:"
echo "  pkill -f 'gazebo\|gz sim\|rosbridge\|ros2 launch\|http.server'"
echo ""
echo "Or run: ./stop_everything.sh"
echo ""
echo -e "${GREEN}🎉 Ready for demo!${NC}"

