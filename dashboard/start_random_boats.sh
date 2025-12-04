#!/bin/bash
# Start Random Boats Spawner with proper ROS environment

# Source ROS
source /opt/ros/jazzy/setup.bash
if [ -d "$HOME/vrx_ws" ]; then
    source $HOME/vrx_ws/install/setup.bash 2>/dev/null || true
fi

# Explicitly set PYTHONPATH (needed when running in background)
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:${PYTHONPATH}"

cd /home/mazhar/final_stand/vrx/dashboard
exec /usr/bin/python3 random_boats_spawner.py
