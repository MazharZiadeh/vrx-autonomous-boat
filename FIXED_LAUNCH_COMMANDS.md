# 🔧 FIXED Launch Commands

## Problem: Boat Not Spawned

The issue is that `vrx_environment.launch.py` launches the world but doesn't automatically spawn a boat. You need to spawn it separately or use a config file.

## ✅ Solution 1: Use spawn.launch.py (Recommended)

**Terminal 1: Launch World First**
```bash
cd /home/mazhar/vrx_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Launch just the world (no boat yet)
ros2 launch vrx_gz vrx_environment.launch.py world:=sydney_regatta sim_mode:=bridge
```

**Wait 10 seconds**, then in **Terminal 2: Spawn Boat**
```bash
cd /home/mazhar/vrx_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Spawn WAM-V boat
ros2 launch vrx_gz spawn.launch.py \
  world:=sydney_regatta \
  name:=wamv \
  model:=wamv \
  x:=0 y:=0 z:=0 \
  sim_mode:=full
```

## ✅ Solution 2: Single Command (Easier)

**Terminal 1: Launch Everything**
```bash
cd /home/mazhar/vrx_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Launch world
ros2 launch vrx_gz vrx_environment.launch.py world:=sydney_regatta sim_mode:=bridge &
sleep 10

# Spawn boat
ros2 launch vrx_gz spawn.launch.py \
  world:=sydney_regatta \
  name:=wamv \
  model:=wamv \
  x:=0 y:=0 z:=0 \
  sim_mode:=full
```

## ✅ Solution 3: Create Config File (Best for Demo)

Create a config file that spawns the boat automatically:

```bash
# Create config file
cat > /tmp/wamv_config.yaml << EOF
wamv:
  model: wamv
  position: [0, 0, 0, 0, 0, 0]
EOF

# Launch with config
ros2 launch vrx_gz vrx_environment.launch.py \
  world:=sydney_regatta \
  sim_mode:=full \
  config_file:=/tmp/wamv_config.yaml
```

## 🔍 Verify Boat is Spawned

After launching, check topics:
```bash
ros2 topic list | grep wamv
```

You should see:
- `/wamv/pose`
- `/wamv/sensors/gps/gps/fix`
- `/wamv/thrusters/left/thrust`
- `/wamv/thrusters/right/thrust`

## 🎯 Quick Test

Once boat is spawned:
```bash
# Check pose
ros2 topic echo /wamv/pose --once

# Should output pose data, not "topic does not appear to be published"
```

