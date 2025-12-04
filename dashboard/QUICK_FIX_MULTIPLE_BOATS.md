# 🚤 Quick Fix: Multiple Boats Problem

## 🔍 Problem

You have **TWO boats** in Gazebo:
- WAM-V
- wavm (or similar)

**The autonomy node is controlling `/wamv/thrusters/*`** - but which boat is that?

## ✅ Quick Fix: Clean Restart

### Option 1: One Command (Easiest)

```bash
cd ~/final_stand/vrx/dashboard
./CLEAN_RESTART.sh
```

This will:
1. Kill everything (including extra boats)
2. Restart with ONE boat
3. Launch everything else

### Option 2: Manual Clean Restart

```bash
# 1. Kill everything
pkill -f "gazebo\|gz sim\|ros2 launch\|spawn"

# 2. Wait
sleep 5

# 3. Restart clean
cd ~/final_stand/vrx/dashboard
./LAUNCH_FULL_SYSTEM.sh
```

## 🎯 Which Boat is Being Controlled?

The autonomy node publishes to:
- `/wamv/thrusters/left/thrust`
- `/wamv/thrusters/right/thrust`

**To test which boat responds:**

```bash
source ~/vrx_ws/install/setup.bash

# Send forward command
ros2 topic pub /wamv/thrusters/left/thrust std_msgs/msg/Float64 "{data: 100.0}" --once
ros2 topic pub /wamv/thrusters/right/thrust std_msgs/msg/Float64 "{data: 100.0}" --once
```

**Watch Gazebo** - which boat moves?

- If **WAM-V** moves: Good! That's the right boat
- If **wavm** moves: Wrong boat - need to check topic names
- If **both** move: Both boats are listening to same topics (bad!)

## 🔍 Check All Thrust Topics

```bash
ros2 topic list | grep thrust
```

You might see:
- `/wamv/thrusters/left/thrust` ← Autonomy uses this
- `/wavm/thrusters/left/thrust` ← Different boat?
- Other variations?

## ✅ Solution

**Best approach:** Clean restart with ONE boat

```bash
cd ~/final_stand/vrx/dashboard
./CLEAN_RESTART.sh
```

This ensures:
- ✅ Only ONE boat spawned
- ✅ Autonomy controls the right boat
- ✅ No confusion

## 🐛 If Still Have Two Boats After Restart

1. **In Gazebo:** Right-click on extra boat → Delete
2. **Or:** Kill Gazebo completely and restart

## 📝 Prevention

**Never run `spawn.launch.py` twice!**

The `LAUNCH_FULL_SYSTEM.sh` script handles this automatically.

---

**Run `./CLEAN_RESTART.sh` and you'll have ONE boat!** 🚤

