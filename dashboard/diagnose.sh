#!/bin/bash

echo "=========================================="
echo "🔍 VRX SYSTEM DIAGNOSTIC REPORT"
echo "=========================================="
echo ""

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Track issues
ISSUES=0
WARNINGS=0

# ============================================
# 1. CHECK ROS 2 INSTALLATION
# ============================================
echo "1️⃣  Checking ROS 2 Installation..."
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    echo -e "${GREEN}✓${NC} ROS 2 Jazzy found"
    source /opt/ros/jazzy/setup.bash
elif [ -f "/opt/ros/humble/setup.bash" ]; then
    echo -e "${GREEN}✓${NC} ROS 2 Humble found"
    source /opt/ros/humble/setup.bash
else
    echo -e "${RED}✗${NC} ROS 2 not found!"
    ISSUES=$((ISSUES + 1))
    exit 1
fi

# ============================================
# 2. CHECK VRX WORKSPACE
# ============================================
echo ""
echo "2️⃣  Checking VRX Workspace..."
if [ -d "$HOME/vrx_ws" ]; then
    echo -e "${GREEN}✓${NC} VRX workspace found at $HOME/vrx_ws"
    cd $HOME/vrx_ws
    if [ -f "install/setup.bash" ]; then
        source install/setup.bash
        echo -e "${GREEN}✓${NC} VRX workspace built"
    else
        echo -e "${YELLOW}⚠${NC} VRX workspace not built (run: colcon build)"
        WARNINGS=$((WARNINGS + 1))
    fi
else
    echo -e "${RED}✗${NC} VRX workspace not found at $HOME/vrx_ws"
    ISSUES=$((ISSUES + 1))
fi

# ============================================
# 3. CHECK ROSBRIDGE INSTALLATION
# ============================================
echo ""
echo "3️⃣  Checking rosbridge Installation..."
if ros2 pkg list | grep -q "rosbridge"; then
    echo -e "${GREEN}✓${NC} rosbridge_suite installed"
    ros2 pkg list | grep rosbridge | while read pkg; do
        echo "   - $pkg"
    done
else
    echo -e "${RED}✗${NC} rosbridge_suite not installed"
    echo "   Install with: sudo apt install ros-$(ros2 pkg list | head -1 | cut -d'_' -f2)-rosbridge-suite"
    ISSUES=$((ISSUES + 1))
fi

# ============================================
# 4. CHECK RUNNING PROCESSES
# ============================================
echo ""
echo "4️⃣  Checking Running Processes..."
GAZEBO_RUNNING=$(ps aux | grep -E "gazebo|gz sim" | grep -v grep | wc -l)
ROS2_RUNNING=$(ps aux | grep "ros2" | grep -v grep | wc -l)
ROSBRIDGE_RUNNING=$(ps aux | grep "rosbridge" | grep -v grep | wc -l)

if [ $GAZEBO_RUNNING -gt 0 ]; then
    echo -e "${GREEN}✓${NC} Gazebo is running ($GAZEBO_RUNNING processes)"
else
    echo -e "${YELLOW}⚠${NC} Gazebo is not running"
    WARNINGS=$((WARNINGS + 1))
fi

if [ $ROS2_RUNNING -gt 0 ]; then
    echo -e "${GREEN}✓${NC} ROS 2 nodes are running ($ROS2_RUNNING processes)"
else
    echo -e "${YELLOW}⚠${NC} No ROS 2 nodes detected"
    WARNINGS=$((WARNINGS + 1))
fi

if [ $ROSBRIDGE_RUNNING -gt 0 ]; then
    echo -e "${GREEN}✓${NC} rosbridge is running ($ROSBRIDGE_RUNNING processes)"
else
    echo -e "${YELLOW}⚠${NC} rosbridge is not running"
    WARNINGS=$((WARNINGS + 1))
fi

# ============================================
# 5. CHECK PORT 9090 (rosbridge)
# ============================================
echo ""
echo "5️⃣  Checking Port 9090 (rosbridge)..."
if lsof -i :9090 > /dev/null 2>&1; then
    echo -e "${GREEN}✓${NC} Port 9090 is in use (rosbridge likely running)"
    lsof -i :9090 | tail -1
else
    echo -e "${YELLOW}⚠${NC} Port 9090 is not in use"
    WARNINGS=$((WARNINGS + 1))
fi

# ============================================
# 6. LIST AVAILABLE TOPICS
# ============================================
echo ""
echo "6️⃣  Listing Available ROS Topics..."
if command -v ros2 &> /dev/null; then
    echo "Available topics:"
    TOPICS=$(ros2 topic list 2>/dev/null)
    if [ -z "$TOPICS" ]; then
        echo -e "${YELLOW}⚠${NC} No topics found (VRX may not be running)"
        WARNINGS=$((WARNINGS + 1))
    else
        echo "$TOPICS" | while read topic; do
            echo "   - $topic"
        done
        
        # Check for specific topics we need
        echo ""
        echo "Checking required topics:"
        
        REQUIRED_TOPICS=(
            "/wamv/sensors/gps/gps/fix"
            "/wamv/sensors/imu/imu/data"
            "/wamv/pose"
        )
        
        for topic in "${REQUIRED_TOPICS[@]}"; do
            if echo "$TOPICS" | grep -q "$topic"; then
                echo -e "${GREEN}✓${NC} $topic"
            else
                echo -e "${RED}✗${NC} $topic (NOT FOUND)"
                ISSUES=$((ISSUES + 1))
            fi
        done
        
        # Check optional topics
        echo ""
        echo "Checking optional topics:"
        OPTIONAL_TOPICS=(
            "/vrx/debug/wind/direction"
            "/vrx/debug/wind/speed"
            "/wamv/mission/current_waypoint"
        )
        
        for topic in "${OPTIONAL_TOPICS[@]}"; do
            if echo "$TOPICS" | grep -q "$topic"; then
                echo -e "${GREEN}✓${NC} $topic"
            else
                echo -e "${YELLOW}⚠${NC} $topic (not available)"
            fi
        done
    fi
else
    echo -e "${RED}✗${NC} ros2 command not found"
    ISSUES=$((ISSUES + 1))
fi

# ============================================
# 7. TEST GPS TOPIC DATA
# ============================================
echo ""
echo "7️⃣  Testing GPS Topic Data..."
if echo "$TOPICS" | grep -q "/wamv/sensors/gps/gps/fix"; then
    echo "Attempting to read GPS data (timeout: 5 seconds)..."
    GPS_DATA=$(timeout 5 ros2 topic echo /wamv/sensors/gps/gps/fix --once 2>&1)
    if [ $? -eq 0 ] && [ ! -z "$GPS_DATA" ]; then
        echo -e "${GREEN}✓${NC} GPS topic is publishing data"
        echo "Sample data:"
        echo "$GPS_DATA" | head -10 | sed 's/^/   /'
    else
        echo -e "${YELLOW}⚠${NC} GPS topic exists but no data received (boat may not be spawned)"
        WARNINGS=$((WARNINGS + 1))
    fi
else
    echo -e "${RED}✗${NC} GPS topic not found"
    ISSUES=$((ISSUES + 1))
fi

# ============================================
# 8. TEST IMU TOPIC DATA
# ============================================
echo ""
echo "8️⃣  Testing IMU Topic Data..."
if echo "$TOPICS" | grep -q "/wamv/sensors/imu/imu/data"; then
    echo "Attempting to read IMU data (timeout: 5 seconds)..."
    IMU_DATA=$(timeout 5 ros2 topic echo /wamv/sensors/imu/imu/data --once 2>&1)
    if [ $? -eq 0 ] && [ ! -z "$IMU_DATA" ]; then
        echo -e "${GREEN}✓${NC} IMU topic is publishing data"
    else
        echo -e "${YELLOW}⚠${NC} IMU topic exists but no data received"
        WARNINGS=$((WARNINGS + 1))
    fi
else
    echo -e "${RED}✗${NC} IMU topic not found"
    ISSUES=$((ISSUES + 1))
fi

# ============================================
# 9. CHECK DASHBOARD FILES
# ============================================
echo ""
echo "9️⃣  Checking Dashboard Files..."
DASHBOARD_DIR="$HOME/final_stand/vrx/dashboard"
if [ -f "$DASHBOARD_DIR/dashboard.html" ]; then
    echo -e "${GREEN}✓${NC} dashboard.html found"
else
    echo -e "${RED}✗${NC} dashboard.html not found at $DASHBOARD_DIR"
    ISSUES=$((ISSUES + 1))
fi

# ============================================
# 10. SUMMARY
# ============================================
echo ""
echo "=========================================="
echo "📊 DIAGNOSTIC SUMMARY"
echo "=========================================="
echo -e "Critical Issues: ${RED}$ISSUES${NC}"
echo -e "Warnings: ${YELLOW}$WARNINGS${NC}"
echo ""

if [ $ISSUES -eq 0 ]; then
    echo -e "${GREEN}✓ System appears ready!${NC}"
    exit 0
else
    echo -e "${RED}✗ System has issues that need fixing${NC}"
    exit 1
fi

