#!/bin/bash

# ============================================
# MASTER LAUNCH SCRIPT - Complete System
# ============================================
# Launches everything needed for full demo

set -e

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo "=========================================="
echo "🚤 LAUNCHING COMPLETE VRX SYSTEM"
echo "=========================================="
echo ""

# Step 1: Kill everything
echo -e "${YELLOW}1️⃣  Cleaning up existing processes...${NC}"
pkill -f "gazebo\|gz sim\|rosbridge\|ros2 launch\|waypoint_navigator\|mission_manager\|http.server" 2>/dev/null || true
sleep 3
echo -e "${GREEN}✓ Cleaned up${NC}"
echo ""

# Step 2: Source ROS
echo -e "${YELLOW}2️⃣  Sourcing ROS 2...${NC}"
source /opt/ros/jazzy/setup.bash
if [ -d "$HOME/vrx_ws" ]; then
    cd $HOME/vrx_ws
    source install/setup.bash 2>/dev/null || true
fi
echo -e "${GREEN}✓ ROS sourced${NC}"
echo ""

# Step 3: Launch VRX
echo -e "${YELLOW}3️⃣  Launching VRX Environment...${NC}"
cd $HOME/vrx_ws
ros2 launch vrx_gz competition.launch.py world:=sydney_regatta > /tmp/vrx_full.log 2>&1 &
VRX_PID=$!
echo "   VRX PID: $VRX_PID"
echo "   Waiting 25 seconds for world to load..."
sleep 25
echo -e "${GREEN}✓ VRX launched${NC}"
echo ""

# Step 4: Spawn Boat
echo -e "${YELLOW}4️⃣  Spawning WAM-V Boat...${NC}"
cd $HOME/vrx_ws
ros2 launch vrx_gz spawn.launch.py > /tmp/spawn_full.log 2>&1 &
SPAWN_PID=$!
echo "   Spawn PID: $SPAWN_PID"
echo "   Waiting 12 seconds for boat to spawn..."
sleep 12
echo -e "${GREEN}✓ Boat spawned${NC}"
echo ""

# Step 5: Launch rosbridge
echo -e "${YELLOW}5️⃣  Launching rosbridge...${NC}"
cd $HOME/final_stand/vrx/dashboard
ros2 launch $(pwd)/rosbridge_fixed.launch.py > /tmp/rosbridge_full.log 2>&1 &
ROSBRIDGE_PID=$!
echo "   rosbridge PID: $ROSBRIDGE_PID"
echo "   Waiting 5 seconds for rosbridge to start..."
sleep 5
if lsof -i :9090 > /dev/null 2>&1; then
    echo -e "${GREEN}✓ rosbridge running on port 9090${NC}"
else
    echo -e "${RED}✗ rosbridge may have failed - check /tmp/rosbridge_full.log${NC}"
fi
echo ""

# Step 6: Launch Dashboard Server
echo -e "${YELLOW}6️⃣  Starting Dashboard Server...${NC}"
cd $HOME/final_stand/vrx/dashboard
python3 -m http.server 8000 > /tmp/dashboard_full.log 2>&1 &
DASHBOARD_PID=$!
echo "   Dashboard PID: $DASHBOARD_PID"
sleep 2
if lsof -i :8000 > /dev/null 2>&1; then
    echo -e "${GREEN}✓ Dashboard server running on port 8000${NC}"
else
    echo -e "${RED}✗ Dashboard server may have failed${NC}"
fi
echo ""

# Step 7: Launch Autonomy Node
echo -e "${YELLOW}7️⃣  Launching Autonomy Node...${NC}"
cd $HOME/vrx_ws
ros2 launch wamv_autonomy autonomy.launch.py \
    heading_tolerance:=15.0 \
    waypoint_threshold:=10.0 \
    thrust_power:=50.0 > /tmp/autonomy_full.log 2>&1 &
AUTONOMY_PID=$!
echo "   Autonomy PID: $AUTONOMY_PID"
sleep 3
echo -e "${GREEN}✓ Autonomy node launched${NC}"
echo ""

# Step 8: Open Browser
echo -e "${YELLOW}8️⃣  Opening Dashboard in Browser...${NC}"
sleep 2
if command -v xdg-open &> /dev/null; then
    xdg-open "http://localhost:8000/dashboard.html" 2>/dev/null &
elif command -v gnome-open &> /dev/null; then
    gnome-open "http://localhost:8000/dashboard.html" 2>/dev/null &
fi
echo -e "${GREEN}✓ Browser opened${NC}"
echo ""

# Summary
echo "=========================================="
echo "✅ SYSTEM FULLY LAUNCHED!"
echo "=========================================="
echo ""
echo "Process IDs:"
echo "  VRX:        $VRX_PID"
echo "  Spawn:      $SPAWN_PID"
echo "  rosbridge:  $ROSBRIDGE_PID"
echo "  Dashboard:  $DASHBOARD_PID"
echo "  Autonomy:   $AUTONOMY_PID"
echo ""
echo "Dashboard: http://localhost:8000/dashboard.html"
echo ""
echo "Logs:"
echo "  VRX:       /tmp/vrx_full.log"
echo "  Spawn:     /tmp/spawn_full.log"
echo "  rosbridge: /tmp/rosbridge_full.log"
echo "  Dashboard: /tmp/dashboard_full.log"
echo "  Autonomy:  /tmp/autonomy_full.log"
echo ""
echo "To stop everything:"
echo "  pkill -f 'gazebo\|gz sim\|rosbridge\|ros2 launch\|http.server'"
echo ""
echo "Or run: cd ~/final_stand/vrx/dashboard && ./stop_everything.sh"
echo ""
echo -e "${GREEN}🎉 Ready for testing!${NC}"
echo ""
echo "Next: Send a mission to see waypoints on dashboard!"
echo "  cd ~/final_stand/vrx/dashboard"
echo "  ./demo_mission.sh"
echo ""

