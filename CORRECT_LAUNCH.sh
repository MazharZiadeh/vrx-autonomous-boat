#!/bin/bash
# CORRECT Launch Command - Spawns WAM-V boat

cd /home/mazhar/vrx_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "=========================================="
echo "🚤 Launching VRX with WAM-V Boat"
echo "=========================================="
echo ""
echo "This will:"
echo "  1. Launch Gazebo with ocean world"
echo "  2. Spawn WAM-V boat at origin"
echo "  3. Bridge all topics to ROS 2"
echo ""
echo "⏳ Wait 15-20 seconds for full initialization..."
echo ""

# Launch VRX environment first
ros2 launch vrx_gz vrx_environment.launch.py world:=sydney_regatta sim_mode:=full &
VRX_PID=$!

# Wait a bit for world to load
sleep 10

# Spawn WAM-V boat
ros2 launch vrx_gz spawn.launch.py \
  world:=sydney_regatta \
  name:=wamv \
  model:=wamv \
  x:=0 y:=0 z:=0 \
  sim_mode:=full

wait $VRX_PID

