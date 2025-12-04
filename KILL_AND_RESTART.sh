#!/bin/bash
# Kill all VRX processes and restart cleanly

echo "Killing all VRX/Gazebo processes..."
pkill -f "gz sim"
pkill -f "vrx"
pkill -f "ros2 launch"
pkill -f "parameter_bridge"
pkill -f "robot_state_publisher"
sleep 2

echo "Done! Now run the commands in order:"
echo ""
echo "TERMINAL 1:"
echo "cd /home/mazhar/vrx_ws"
echo "source /opt/ros/jazzy/setup.bash"
echo "source install/setup.bash"
echo "ros2 launch vrx_gz vrx_environment.launch.py world:=sydney_regatta sim_mode:=bridge"
echo ""
echo "TERMINAL 2 (after 15 seconds):"
echo "cd /home/mazhar/vrx_ws"
echo "source /opt/ros/jazzy/setup.bash"
echo "source install/setup.bash"
echo "ros2 launch vrx_gz spawn.launch.py world:=sydney_regatta name:=wamv model:=wamv x:=0 y:=0 z:=0 sim_mode:=full"

