#!/bin/bash
# Spawn 20 WAM-V boats (same as main boat) using the EXACT same method

set +e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo "=========================================="
echo "🚤 Spawning 20 WAM-V Boats (Same as Main)"
echo "=========================================="
echo ""

# Source ROS
source /opt/ros/jazzy/setup.bash
if [ -d "$HOME/vrx_ws" ]; then
    source $HOME/vrx_ws/install/setup.bash 2>/dev/null || true
fi

cd $HOME/vrx_ws

# Wait for Gazebo
echo "Waiting for Gazebo..."
for i in {1..30}; do
    if ros2 service list 2>/dev/null | grep -q "/world/simple_demo/create"; then
        echo "✅ Gazebo ready"
        break
    fi
    sleep 1
done

echo ""
echo "Spawning 20 WAM-V boats..."
echo ""

spawned=0

# Spawn 20 boats using the SAME method as main boat
for i in {1..20}; do
    # Random position in water (avoid main boat at -532, 162)
    x=$(python3 -c "import random; print(f'{random.uniform(-600, -400):.1f}')")
    y=$(python3 -c "import random; print(f'{random.uniform(100, 250):.1f}')")
    
    # Make sure not too close to main boat
    dist=$(python3 -c "import random, math; x=$x; y=$y; print(math.sqrt((x+532)**2 + (y-162)**2))")
    if (( $(echo "$dist < 50" | bc -l) )); then
        # Too close, adjust
        x=$(python3 -c "import random; print(f'{-532 + random.choice([-1,1]) * random.uniform(60, 100):.1f}')")
        y=$(python3 -c "import random; print(f'{162 + random.choice([-1,1]) * random.uniform(60, 100):.1f}')")
    fi
    
    boat_name="wamv_boat_$i"
    
    echo -n "Boat $i/20 ($boat_name)... "
    
    # Use EXACT same spawn command as main boat, just different name and position
    ros2 launch vrx_gz spawn.launch.py \
        world:=sydney_regatta \
        name:="$boat_name" \
        model:=wamv \
        x:=$x \
        y:=$y \
        z:=0 \
        sim_mode:=sim > /tmp/spawn_${i}.log 2>&1 &
    
    SPAWN_PID=$!
    sleep 2
    
    if ps -p $SPAWN_PID > /dev/null 2>&1 || grep -q "success" /tmp/spawn_${i}.log 2>/dev/null; then
        echo "✅"
        ((spawned++))
        kill $SPAWN_PID 2>/dev/null
    else
        echo "⚠️"
    fi
    
    sleep 0.3
done

echo ""
echo "✅ Done! Spawned $spawned/20 WAM-V boats"
echo ""
echo "Starting GPS publisher for spawned boats..."
cd /home/mazhar/final_stand/vrx/dashboard
python3 spawned_boats_publisher.py > /tmp/spawned_boats_publisher.log 2>&1 &
SPAWNED_PUB_PID=$!
echo "   GPS Publisher PID: $SPAWNED_PUB_PID"
echo ""
echo "✅ Spawned boats should now appear on the dashboard (red markers)"
echo ""
