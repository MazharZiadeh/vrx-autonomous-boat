# 🧪 Testing Dashboard

## Quick Test (3 Steps)

### Step 1: Start rosbridge
```bash
cd /home/mazhar/vrx_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

### Step 2: Start web server
```bash
cd /home/mazhar/final_stand/vrx/dashboard
python3 -m http.server 8000
```

### Step 3: Open dashboard
**http://localhost:8000/dashboard.html**

## ✅ What to Check

1. **Map loads** - Should see map centered on Sydney Regatta
2. **Boat marker** - Blue triangle at center
3. **MiniMap** - Bottom-right corner
4. **Test waypoints** - Red numbered markers
5. **Test path** - Orange dashed line
6. **Console logs** - Open F12, should see initialization messages

## 🔍 Debugging

### Open Browser Console (F12)

You should see:
```
🚤 ==========================================
🚤 WAM-V Dashboard Initializing...
🚤 ==========================================
🗺️ Initializing maps...
✅ Main map initialized at: [-33.722, 150.674]
✅ Boat marker added
✅ MiniMap initialized
🔌 Connecting to ROS at ws://localhost:9090...
```

### When ROS Connects:
```
✅ Connected to ROS!
📡 Subscribing to ROS topics...
✅ Subscribed to /wamv/sensors/gps/gps/fix
✅ Subscribed to /wamv/sensors/imu/imu/data
✅ Subscribed to /wamv/pose
✅ Subscribed to /vrx/debug/wind/direction
✅ Subscribed to /vrx/debug/wind/speed
✅ All topics subscribed
```

### When Data Arrives:
```
📍 GPS Message: {latitude: -33.722, longitude: 150.674, ...}
🧭 IMU Message: {orientation: {...}, ...}
🎯 Pose Message: {position: {...}, orientation: {...}}
💨 Wind Direction: {data: 0.5}
💨 Wind Speed: {data: 2.0}
```

## 🐛 Troubleshooting

**Map not loading?**
- Check internet (needs OpenStreetMap tiles)
- Check browser console for errors

**"Disconnected" status?**
- Verify rosbridge is running: `ps aux | grep rosbridge`
- Check port: `netstat -tuln | grep 9090`

**No console logs?**
- Make sure browser console is open (F12)
- Check if JavaScript is enabled

**Boat not moving?**
- Check if VRX is running with boat spawned
- Verify topics: `ros2 topic list | grep wamv`
- Check console for ROS messages

## 📊 Expected Behavior

1. **Initial state**: Boat marker at center, static
2. **After ROS connects**: Status changes to "Connected"
3. **When GPS data arrives**: Boat marker moves, path trail appears
4. **When heading data arrives**: Boat marker rotates
5. **Telemetry updates**: All values update in real-time
