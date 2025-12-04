# 🔧 FIX DASHBOARD CONNECTION

## Problem: Dashboard Shows "Disconnected ✗"

This means rosbridge WebSocket server is not running on port 9090.

## ✅ Quick Fix

### Option 1: Use the Launch Script (Recommended)
```bash
cd ~/final_stand/vrx/dashboard
./launch_everything.sh
```

This starts everything including rosbridge.

### Option 2: Start rosbridge Manually

**Terminal 1: Start rosbridge**
```bash
source /opt/ros/jazzy/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

You should see:
```
[INFO] [rosbridge_websocket]: Rosbridge WebSocket server started on port 9090
```

**Terminal 2: Refresh Dashboard**
- Open: http://localhost:8000/dashboard.html
- Press F5 to refresh
- Should now show "Connected ✓"

## 🔍 Verify Connection

### Check if rosbridge is running:
```bash
lsof -i :9090
```

Should show a process using port 9090.

### Check browser console:
1. Open dashboard
2. Press F12 (open developer tools)
3. Go to Console tab
4. Look for:
   - ✅ "Connected to ROS!" = Working!
   - ❌ "ROS Error" = Connection failed

## 🐛 Common Issues

### Issue 1: Port 9090 Already in Use
```bash
# Kill existing rosbridge
pkill -f rosbridge
# Wait 2 seconds
sleep 2
# Start again
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

### Issue 2: ROS Not Sourced
```bash
source /opt/ros/jazzy/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

### Issue 3: VRX Not Running
rosbridge can run without VRX, but you won't get data. Make sure VRX is running:
```bash
# Check if VRX topics exist
ros2 topic list | grep wamv
```

## ✅ Success Indicators

When working, you should see:
1. Dashboard: "Connected ✓" (green)
2. Topics: "5 / 10" (or similar numbers)
3. Browser console: "✅ Connected to ROS!"
4. Data updating: Position, heading, etc.

## 🚀 Full System Launch

If you want everything at once:
```bash
cd ~/final_stand/vrx/dashboard
./launch_everything.sh
```

This launches:
- VRX environment
- WAM-V boat
- rosbridge
- Dashboard server
- Opens browser

