# 🚀 QUICK LAUNCH - Complete System

## One Command to Launch Everything

```bash
cd ~/final_stand/vrx/dashboard
./LAUNCH_FULL_SYSTEM.sh
```

This will:
1. ✅ Kill all existing processes
2. ✅ Launch VRX environment (25s wait)
3. ✅ Spawn WAM-V boat (12s wait)
4. ✅ Start rosbridge (5s wait)
5. ✅ Start dashboard server
6. ✅ Launch autonomy node
7. ✅ Open browser automatically

**Total time: ~45 seconds**

## What You'll See

After launch:
- ✅ Gazebo window opens with ocean and boat
- ✅ Browser opens to dashboard
- ✅ Dashboard shows "Connected ✓"
- ✅ Boat position appears on map
- ✅ Ready to send missions!

## Send Test Mission

After system is launched:

```bash
cd ~/final_stand/vrx/dashboard
./demo_mission.sh
```

Or manually:
```bash
source ~/vrx_ws/install/setup.bash

ros2 topic pub /wamv/mission/waypoints geometry_msgs/msg/PoseArray "
header:
  frame_id: 'map'
poses:
- position: {x: 150.6740, y: -33.7228, z: 0.0}
- position: {x: 150.6745, y: -33.7228, z: 0.0}
- position: {x: 150.6745, y: -33.7233, z: 0.0}
- position: {x: 150.6740, y: -33.7233, z: 0.0}
" --once
```

## Stop Everything

```bash
cd ~/final_stand/vrx/dashboard
./stop_everything.sh
```

Or:
```bash
pkill -f 'gazebo\|gz sim\|rosbridge\|ros2 launch\|http.server'
```

## Verify Everything is Running

```bash
# Check processes
ps aux | grep -E "gazebo|rosbridge|http.server|waypoint_navigator" | grep -v grep

# Check ports
lsof -i :9090  # rosbridge
lsof -i :8000  # dashboard

# Check topics
ros2 topic list | grep -E "wamv|mission"
```

## Troubleshooting

### Gazebo Not Opening
- Check logs: `tail -50 /tmp/vrx_full.log`
- May need to wait longer (Gazebo is slow to start)

### Dashboard Not Connecting
- Check rosbridge: `lsof -i :9090`
- Check logs: `tail -50 /tmp/rosbridge_full.log`
- Refresh browser (F5)

### Boat Not Spawning
- Check logs: `tail -50 /tmp/spawn_full.log`
- Wait longer (spawn can take 15-20 seconds)

### Autonomy Not Working
- Check logs: `tail -50 /tmp/autonomy_full.log`
- Verify topics: `ros2 topic list | grep mission`

---

**One command, complete system!** 🚤

