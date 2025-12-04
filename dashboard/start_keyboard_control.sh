#!/bin/bash
# Start Keyboard Control for MAIN BOAT
# This script launches arrow key control for the WAM-V main boat

set +e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo "=========================================="
echo "⌨️  Starting Keyboard Control (Main Boat)"
echo "=========================================="
echo ""

# Source ROS
source /opt/ros/jazzy/setup.bash
if [ -d "$HOME/vrx_ws" ]; then
    source $HOME/vrx_ws/install/setup.bash 2>/dev/null || true
fi

cd /home/mazhar/final_stand/vrx/dashboard

# Check if ROS is running
if ! ros2 node list > /dev/null 2>&1; then
    echo -e "${RED}✗ ROS 2 is not running!${NC}"
    echo "   Please start Gazebo first:"
    echo "   ros2 launch vrx_gz competition.launch.py world:=sydney_regatta"
    exit 1
fi

# Check if main boat exists
if ! ros2 topic list | grep -q "/wamv/thrusters"; then
    echo -e "${YELLOW}⚠ Main boat topics not found${NC}"
    echo "   Make sure the main boat is spawned in Gazebo"
    echo ""
fi

echo -e "${GREEN}✅ ROS environment ready${NC}"
echo ""
echo "=========================================="
echo "⌨️  ARROW KEY CONTROLS"
echo "=========================================="
echo ""
echo "  ↑  : Forward"
echo "  ↓  : Backward"
echo "  ←  : Turn Left"
echo "  →  : Turn Right"
echo "  Space : Stop"
echo "  q  : Quit"
echo ""
echo "=========================================="
echo ""
echo -e "${YELLOW}Starting arrow key control...${NC}"
echo "   (Make sure this terminal is focused!)"
echo ""

# Make script executable
chmod +x arrow_key_teleop.py

# Run arrow key control
python3 arrow_key_teleop.py

