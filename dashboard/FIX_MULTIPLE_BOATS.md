# 🔧 Fix: Multiple Boats Spawned

## 🔍 Problem

You have TWO boats in Gazebo:
- WAM-V
- wavm (or similar)

This happens when `spawn.launch.py` is run multiple times.

## ✅ Solution: Kill Extra Boats

### Step 1: Kill All Spawn Processes

```bash
pkill -f "spawn.launch"
pkill -f "gz model"
```

### Step 2: Kill Everything and Restart Clean

```bash
# Kill everything
pkill -f "gazebo\|gz sim\|ros2 launch\|spawn"

# Wait a moment
sleep 3

# Restart VRX (this will spawn ONE boat)
cd ~/vrx_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch vrx_gz competition.launch.py world:=sydney_regatta
```

**Wait 20 seconds**, then in another terminal:

```bash
# Spawn boat ONCE
cd ~/vrx_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch vrx_gz spawn.launch.py
```

**Only run spawn ONCE!**

### Step 3: Verify Only One Boat

```bash
# Check topics - should see /wamv/ topics
ros2 topic list | grep wamv | wc -l

# Should see many /wamv/ topics (GPS, IMU, thrusters, etc.)
```

## 🎯 Which Boat is Being Controlled?

The autonomy node publishes to:
- `/wamv/thrusters/left/thrust`
- `/wamv/thrusters/right/thrust`

**Check which boat responds:**

1. **Manual test:**
```bash
ros2 topic pub /wamv/thrusters/left/thrust std_msgs/msg/Float64 "{data: 50.0}" --once
ros2 topic pub /wamv/thrusters/right/thrust std_msgs/msg/Float64 "{data: 50.0}" --once
```

2. **Watch Gazebo** - which boat moves?

3. **If wrong boat moves:**
   - Check if there's a different topic name
   - List all thrust topics: `ros2 topic list | grep thrust`

## 🚀 Clean Restart (Recommended)

```bash
cd ~/final_stand/vrx/dashboard
./LAUNCH_FULL_SYSTEM.sh
```

This script:
- Kills everything first
- Launches VRX (spawns one boat)
- Spawns boat once
- Starts everything else

## 🔍 Check Boat Names in Gazebo

In Gazebo, check the model names:
- Right-click on boat → "Inspect"
- Check the model name
- Should be "wamv" or "WAM-V"

## 📝 Prevention

**Never run `spawn.launch.py` twice!**

If you need to restart:
1. Kill everything first
2. Then launch fresh

## ✅ Quick Fix Right Now

```bash
# 1. Kill everything
pkill -f "gazebo\|gz sim\|ros2 launch\|spawn"

# 2. Wait
sleep 5

# 3. Restart clean
cd ~/final_stand/vrx/dashboard
./LAUNCH_FULL_SYSTEM.sh
```

This will give you ONE boat and everything should work! 🚤

