# 🧹 CLEAN START - One Boat Only

## Step 1: KILL EVERYTHING First

Run this in ANY terminal to kill all processes:
```bash
pkill -f "gz sim"
pkill -f "vrx"
pkill -f "ros2 launch"
pkill -f "parameter_bridge"
sleep 3
```

**Wait 3 seconds** for everything to die.

---

## Step 2: Launch World (Terminal 1)

```bash
cd /home/mazhar/vrx_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch vrx_gz vrx_environment.launch.py world:=sydney_regatta sim_mode:=bridge
```

**Wait 15-20 seconds** for Gazebo to fully load. You should see the ocean world.

---

## Step 3: Spawn ONE Boat (Terminal 2)

**IMPORTANT: Run this ONCE only!**

```bash
cd /home/mazhar/vrx_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch vrx_gz spawn.launch.py world:=sydney_regatta name:=wamv model:=wamv x:=0 y:=0 z:=0 sim_mode:=full
```

**Wait 5-10 seconds** for boat to spawn. You should see "Entity creation successful" message.

**DO NOT RUN THIS COMMAND AGAIN!** If you do, you'll spawn another boat.

---

## Step 4: Verify (Terminal 3)

```bash
cd /home/mazhar/vrx_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 topic list | grep wamv
```

Should see topics. If you see multiple `/wamv/pose` topics, you have multiple boats.

---

## Step 5: Launch Autonomy (Terminal 4)

```bash
cd /home/mazhar/vrx_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run wamv_autonomy autonomy_node
```

---

## 🐛 If You Still See Multiple Boats

1. **Kill everything again:**
   ```bash
   pkill -f "gz sim"
   pkill -f "vrx"
   sleep 3
   ```

2. **Check if any Gazebo is still running:**
   ```bash
   ps aux | grep "gz sim" | grep -v grep
   ```
   Should return nothing.

3. **Start over from Step 2**

---

## ⚠️ Why This Happens

- Running spawn command multiple times = multiple boats
- Multiple Gazebo instances = lag and confusion
- Always kill everything before restarting

