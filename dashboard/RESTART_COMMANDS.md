# 🔄 System Restart Commands

## 🚀 Quick Restart (Recommended)

**Complete system restart (stops everything and starts fresh):**

```bash
cd ~/final_stand/vrx/dashboard
./RESTART_SYSTEM.sh
```

This will:
1. ✅ Stop all processes (Gazebo, ROS, AIS, dashboard)
2. ✅ Clean up ports (9090, 9091, 8000)
3. ✅ Start Gazebo with Sydney Regatta world
4. ✅ Spawn main boat
5. ✅ Start rosbridge server
6. ✅ Start dashboard server

**Access:**
- Dashboard: http://localhost:8000/dashboard.html
- rosbridge: ws://localhost:9090

---

## 🛑 Stop Everything

**Stop all VRX processes:**

```bash
cd ~/final_stand/vrx/dashboard
./STOP_SYSTEM.sh
```

Or manually:
```bash
pkill -f "gz sim\|ros2 launch\|rosbridge\|http.server\|ais_proxy\|arrow_key"
```

---

## 📋 Manual Restart Steps

If you prefer step-by-step control:

### **Step 1: Stop Everything**
```bash
cd ~/final_stand/vrx/dashboard
./STOP_SYSTEM.sh
```

### **Step 2: Start Gazebo**
```bash
source /opt/ros/jazzy/setup.bash
source ~/vrx_ws/install/setup.bash
cd ~/vrx_ws
ros2 launch vrx_gz competition.launch.py world:=sydney_regatta
```

### **Step 3: Spawn Main Boat** (in new terminal)
```bash
source /opt/ros/jazzy/setup.bash
source ~/vrx_ws/install/setup.bash
cd ~/vrx_ws
ros2 launch vrx_gz spawn.launch.py
```

### **Step 4: Start rosbridge** (in new terminal)
```bash
source /opt/ros/jazzy/setup.bash
ros2 run rosbridge_server rosbridge_websocket --ros-args -p port:=9090
```

### **Step 5: Start Dashboard** (in new terminal)
```bash
cd ~/final_stand/vrx/dashboard
python3 -m http.server 8000
```

---

## 🎮 After Restart - Additional Services

### **Start Keyboard Control**
```bash
cd ~/final_stand/vrx/dashboard
./start_keyboard_control.sh
```

### **Start AIS Integration** (optional)
```bash
cd ~/final_stand/vrx/dashboard
./start_ais.sh
```

### **Spawn NPC Boats** (optional)
```bash
cd ~/final_stand/vrx/dashboard
./spawn_wamv_boats.sh
```

---

## 🔍 Verify System Status

### **Check if Gazebo is running:**
```bash
ps aux | grep "gz sim" | grep -v grep
```

### **Check if rosbridge is running:**
```bash
ps aux | grep rosbridge | grep -v grep
```

### **Check if dashboard is running:**
```bash
ps aux | grep "http.server" | grep -v grep
```

### **Check ROS topics:**
```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list | grep wamv
```

### **Check ports:**
```bash
lsof -i :9090  # rosbridge
lsof -i :9091  # AIS proxy
lsof -i :8000  # dashboard
```

---

## 🐛 Troubleshooting

### **Problem: Port already in use**

**Solution:**
```bash
# Kill process on port 9090
lsof -ti :9090 | xargs kill -9

# Kill process on port 8000
lsof -ti :8000 | xargs kill -9

# Kill process on port 9091
lsof -ti :9091 | xargs kill -9
```

### **Problem: Gazebo won't start**

**Check logs:**
```bash
cat /tmp/gazebo.log
```

**Try manual start:**
```bash
source /opt/ros/jazzy/setup.bash
source ~/vrx_ws/install/setup.bash
ros2 launch vrx_gz competition.launch.py world:=sydney_regatta
```

### **Problem: Main boat not spawning**

**Check spawn log:**
```bash
cat /tmp/spawn.log
```

**Try manual spawn:**
```bash
source /opt/ros/jazzy/setup.bash
source ~/vrx_ws/install/setup.bash
ros2 launch vrx_gz spawn.launch.py
```

### **Problem: Dashboard not loading**

**Check if server is running:**
```bash
curl http://localhost:8000/dashboard.html
```

**Restart dashboard:**
```bash
pkill -f "http.server"
cd ~/final_stand/vrx/dashboard
python3 -m http.server 8000
```

---

## 📝 Quick Reference

| Command | Purpose |
|---------|---------|
| `./RESTART_SYSTEM.sh` | Full system restart |
| `./STOP_SYSTEM.sh` | Stop all processes |
| `./start_keyboard_control.sh` | Start keyboard control |
| `./start_ais.sh` | Start AIS integration |
| `./spawn_wamv_boats.sh` | Spawn NPC boats |

---

## ✅ Complete Restart Checklist

After running `./RESTART_SYSTEM.sh`, verify:

- [ ] Gazebo window is open
- [ ] Main boat is visible in Gazebo
- [ ] Dashboard loads: http://localhost:8000/dashboard.html
- [ ] Main boat appears on dashboard map (green marker)
- [ ] GPS data is updating on dashboard
- [ ] rosbridge is running (check browser console for "Connected to ROS!")

---

## 🎯 Typical Workflow

1. **Start system:**
   ```bash
   cd ~/final_stand/vrx/dashboard
   ./RESTART_SYSTEM.sh
   ```

2. **Open dashboard:**
   - Browser: http://localhost:8000/dashboard.html

3. **Control boat** (optional):
   ```bash
   cd ~/final_stand/vrx/dashboard
   ./start_keyboard_control.sh
   ```

4. **Spawn NPC boats** (optional):
   ```bash
   cd ~/final_stand/vrx/dashboard
   ./spawn_wamv_boats.sh
   ```

5. **When done:**
   ```bash
   cd ~/final_stand/vrx/dashboard
   ./STOP_SYSTEM.sh
   ```

---

That's it! Use `./RESTART_SYSTEM.sh` for a complete restart.

