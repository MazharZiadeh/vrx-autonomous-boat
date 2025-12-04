#!/bin/bash

# ============================================
# INTEGRATION TEST SCRIPT
# ============================================
# Tests each component of the system

set -e

# Colors
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

PASS=0
FAIL=0

echo "=========================================="
echo "🧪 VRX SYSTEM INTEGRATION TESTS"
echo "=========================================="
echo ""

# Source ROS
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
elif [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

if [ -d "$HOME/vrx_ws" ]; then
    cd $HOME/vrx_ws
    source install/setup.bash 2>/dev/null || true
fi

# Test 1: VRX Launches
echo "Test 1: VRX Launches"
if ps aux | grep -E "gazebo|gz sim" | grep -v grep > /dev/null; then
    echo -e "${GREEN}✓ PASS${NC}"
    PASS=$((PASS + 1))
else
    echo -e "${RED}✗ FAIL - Gazebo not running${NC}"
    FAIL=$((FAIL + 1))
fi

# Test 2: Boat Spawns
echo ""
echo "Test 2: Boat Spawns"
if ros2 topic list 2>/dev/null | grep -q "/wamv/pose"; then
    echo -e "${GREEN}✓ PASS${NC}"
    PASS=$((PASS + 1))
else
    echo -e "${RED}✗ FAIL - Boat topics not found${NC}"
    FAIL=$((FAIL + 1))
fi

# Test 3: GPS Data Publishes
echo ""
echo "Test 3: GPS Data Publishes"
GPS_DATA=$(timeout 3 ros2 topic echo /wamv/sensors/gps/gps/fix --once 2>&1 | grep -c "latitude" || echo "0")
if [ "$GPS_DATA" -gt 0 ]; then
    echo -e "${GREEN}✓ PASS${NC}"
    PASS=$((PASS + 1))
else
    echo -e "${RED}✗ FAIL - No GPS data${NC}"
    FAIL=$((FAIL + 1))
fi

# Test 4: rosbridge Connects
echo ""
echo "Test 4: rosbridge Connects"
if lsof -i :9090 > /dev/null 2>&1; then
    echo -e "${GREEN}✓ PASS${NC}"
    PASS=$((PASS + 1))
else
    echo -e "${RED}✗ FAIL - Port 9090 not in use${NC}"
    FAIL=$((FAIL + 1))
fi

# Test 5: Dashboard Server Running
echo ""
echo "Test 5: Dashboard Server Running"
if lsof -i :8000 > /dev/null 2>&1; then
    echo -e "${GREEN}✓ PASS${NC}"
    PASS=$((PASS + 1))
else
    echo -e "${RED}✗ FAIL - Port 8000 not in use${NC}"
    FAIL=$((FAIL + 1))
fi

# Test 6: Topics Available
echo ""
echo "Test 6: Required Topics Available"
REQUIRED_TOPICS=(
    "/wamv/sensors/gps/gps/fix"
    "/wamv/sensors/imu/imu/data"
    "/wamv/pose"
)

ALL_TOPICS=$(ros2 topic list 2>/dev/null || echo "")
MISSING=0

for topic in "${REQUIRED_TOPICS[@]}"; do
    if echo "$ALL_TOPICS" | grep -q "$topic"; then
        echo "  ✓ $topic"
    else
        echo "  ✗ $topic (MISSING)"
        MISSING=$((MISSING + 1))
    fi
done

if [ $MISSING -eq 0 ]; then
    echo -e "${GREEN}✓ PASS${NC}"
    PASS=$((PASS + 1))
else
    echo -e "${RED}✗ FAIL - $MISSING topics missing${NC}"
    FAIL=$((FAIL + 1))
fi

# Summary
echo ""
echo "=========================================="
echo "📊 TEST SUMMARY"
echo "=========================================="
echo -e "${GREEN}Passed: $PASS${NC}"
echo -e "${RED}Failed: $FAIL${NC}"
echo ""

if [ $FAIL -eq 0 ]; then
    echo -e "${GREEN}✅ All tests passed!${NC}"
    exit 0
else
    echo -e "${RED}❌ Some tests failed${NC}"
    exit 1
fi

