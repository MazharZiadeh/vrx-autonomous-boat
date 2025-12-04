#!/bin/bash

# Kill extra boat spawns and clean up

echo "🧹 Cleaning up multiple boat spawns..."
echo ""

# Kill spawn processes
pkill -f "spawn.launch" 2>/dev/null
pkill -f "gz model" 2>/dev/null

# Kill Gazebo to remove all models
pkill -f "gazebo\|gz sim" 2>/dev/null

echo "✅ Killed all boat spawns and Gazebo"
echo ""
echo "Now restart with:"
echo "  cd ~/final_stand/vrx/dashboard"
echo "  ./LAUNCH_FULL_SYSTEM.sh"
echo ""

