#!/bin/bash
# Complete setup script for rosbridge and dashboard

echo "=========================================="
echo "🚤 WAM-V Dashboard Complete Setup"
echo "=========================================="
echo ""

# Check ROS 2 version
if [ -f /opt/ros/jazzy/setup.bash ]; then
    ROS_DISTRO="jazzy"
    echo "✅ Detected ROS 2 Jazzy"
elif [ -f /opt/ros/humble/setup.bash ]; then
    ROS_DISTRO="humble"
    echo "✅ Detected ROS 2 Humble"
else
    echo "❌ ROS 2 not found!"
    exit 1
fi

# Install rosbridge
echo ""
echo "Step 1: Installing rosbridge_server..."
if ros2 pkg list | grep -q rosbridge; then
    echo "✅ rosbridge already installed"
else
    sudo apt update
    sudo apt install ros-${ROS_DISTRO}-rosbridge-suite -y
    echo "✅ rosbridge installed"
fi

# Setup workspace
echo ""
echo "Step 2: Setting up workspace..."
cd /home/mazhar/vrx_ws
source /opt/ros/${ROS_DISTRO}/setup.bash
source install/setup.bash 2>/dev/null || echo "⚠️ Workspace not built yet"

echo ""
echo "=========================================="
echo "✅ Setup Complete!"
echo "=========================================="
echo ""
echo "Next steps:"
echo ""
echo "TERMINAL 1: Launch VRX"
echo "  cd /home/mazhar/vrx_ws"
echo "  source /opt/ros/${ROS_DISTRO}/setup.bash"
echo "  source install/setup.bash"
echo "  ros2 launch vrx_gz vrx_environment.launch.py world:=sydney_regatta sim_mode:=bridge"
echo ""
echo "TERMINAL 2: Spawn Boat (after 15 seconds)"
echo "  cd /home/mazhar/vrx_ws"
echo "  source /opt/ros/${ROS_DISTRO}/setup.bash"
echo "  source install/setup.bash"
echo "  ros2 launch vrx_gz spawn.launch.py world:=sydney_regatta name:=wamv model:=wamv x:=0 y:=0 z:=0 sim_mode:=full"
echo ""
echo "TERMINAL 3: Start rosbridge"
echo "  cd /home/mazhar/vrx_ws"
echo "  source /opt/ros/${ROS_DISTRO}/setup.bash"
echo "  source install/setup.bash"
echo "  ros2 launch rosbridge_server rosbridge_websocket_launch.xml"
echo ""
echo "TERMINAL 4: Start Dashboard"
echo "  cd /home/mazhar/final_stand/vrx/dashboard"
echo "  python3 -m http.server 8000"
echo ""
echo "Then open: http://localhost:8000/dashboard.html"
echo ""

