#!/bin/bash
# VRX Launch Commands - Copy and paste these into separate terminals

# ============================================
# TERMINAL 1: Launch VRX Simulation
# ============================================
echo "=== TERMINAL 1: Launch VRX ==="
echo "cd /home/mazhar/vrx_ws"
echo "source /opt/ros/jazzy/setup.bash"
echo "source install/setup.bash"
echo "ros2 launch vrx_gz vrx_environment.launch.py world:=sydney_regatta sim_mode:=full"
echo ""
echo "# Alternative: Simple ocean world"
echo "ros2 launch vrx_gz vrx_environment.launch.py world:=sydney_regatta sim_mode:=full headless:=False"
echo ""

# ============================================
# TERMINAL 2: Monitor Topics
# ============================================
echo "=== TERMINAL 2: Monitor Topics ==="
echo "cd /home/mazhar/vrx_ws"
echo "source /opt/ros/jazzy/setup.bash"
echo "source install/setup.bash"
echo ""
echo "# List all topics"
echo "ros2 topic list"
echo ""
echo "# Check GPS"
echo "ros2 topic echo /wamv/sensors/gps/gps/fix"
echo ""
echo "# Check IMU"
echo "ros2 topic echo /wamv/sensors/imu/imu/data"
echo ""
echo "# Check Pose"
echo "ros2 topic echo /wamv/pose"
echo ""

# ============================================
# TERMINAL 3: Control Boat (Manual Test)
# ============================================
echo "=== TERMINAL 3: Manual Control Test ==="
echo "cd /home/mazhar/vrx_ws"
echo "source /opt/ros/jazzy/setup.bash"
echo "source install/setup.bash"
echo ""
echo "# Publish thrust commands manually"
echo "ros2 topic pub --once /wamv/thrusters/left/thrust std_msgs/msg/Float64 '{data: 500.0}'"
echo "ros2 topic pub --once /wamv/thrusters/right/thrust std_msgs/msg/Float64 '{data: 500.0}'"
echo ""

