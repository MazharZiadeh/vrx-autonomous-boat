#!/bin/bash
# Run Waypoint Follower - Copy and paste these commands

cd /home/mazhar/vrx_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run wamv_autonomy waypoint_follower

