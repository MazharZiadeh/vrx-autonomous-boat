# 🔧 FIX "Disconnected ✗" - Step by Step

## Current Status
- ✅ Dashboard is loading
- ❌ rosbridge not connected (showing "Disconnected ✗")

## Quick Fix (3 Steps)

### Step 1: Open a NEW Terminal
Keep your dashboard open in the browser, but open a new terminal window.

### Step 2: Start rosbridge
In the new terminal, run:
```bash
cd ~/final_stand/vrx/dashboard
source /opt/ros/jazzy/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

**You should see:**
```
[INFO] [rosbridge_websocket]: Rosbridge WebSocket server started on port 9090
```

**Keep this terminal open!** (Don't close it - rosbridge needs to keep running)

### Step 3: Refresh Dashboard
1. Go back to your browser
2. Press **F5** (or Ctrl+R) to refresh
3. Wait 2-3 seconds
4. Should now show: **"Connected ✓"** (green)

## ✅ Success Indicators

When it's working, you'll see:
- **Connection Status**: "Connected ✓" (green text)
- **Topics**: "5 / 10" (or similar numbers)
- **Browser Console** (F12): "✅ Connected to ROS!"

## 🐛 If Still Not Working

### Check Browser Console (F12)
1. Press F12 in browser
2. Go to "Console" tab
3. Look for error messages
4. Common errors:
   - `WebSocket connection failed` = rosbridge not running
   - `Connection refused` = port 9090 not listening
   - `404 Not Found` = wrong URL

### Verify rosbridge is Running
In a terminal:
```bash
# Check if port 9090 is in use
lsof -i :9090

# Check rosbridge process
ps aux | grep rosbridge
```

### Restart Everything
```bash
# Kill everything
pkill -f rosbridge
pkill -f gazebo

# Wait 2 seconds
sleep 2

# Start rosbridge again
cd ~/final_stand/vrx/dashboard
source /opt/ros/jazzy/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

## 🚀 Alternative: Use Launch Script

If you want to start everything at once:
```bash
cd ~/final_stand/vrx/dashboard
./launch_everything.sh
```

This starts:
- VRX environment
- Boat
- rosbridge
- Dashboard server

## 📝 Notes

- **rosbridge must stay running** - don't close the terminal
- **Dashboard auto-retries** - it will try to connect 10 times
- **Refresh browser** if connection doesn't appear after starting rosbridge
- **Check browser console** (F12) for detailed connection logs

