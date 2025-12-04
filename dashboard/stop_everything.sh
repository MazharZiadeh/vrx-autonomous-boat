#!/bin/bash

echo "🛑 Stopping all VRX processes..."

pkill -f "gazebo\|gz sim\|rosbridge\|ros2 launch\|http.server" || true

sleep 2

echo "✅ All processes stopped"

