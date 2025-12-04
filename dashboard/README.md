# 🚤 WAM-V Boat Dashboard

Real-time web dashboard for monitoring autonomous boat in VRX simulation.

## 📋 Prerequisites

1. **rosbridge_server** (ROS 2 package)
2. **Web browser** (Chrome, Firefox, etc.)
3. **VRX simulation running** with boat spawned

## 🚀 Setup Instructions

### Step 1: Install rosbridge_server

```bash
sudo apt install ros-jazzy-rosbridge-suite
```

### Step 2: Launch rosbridge (Terminal 1)

```bash
cd /home/mazhar/vrx_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

**Expected output:**
```
[INFO] [rosbridge_websocket]: Rosbridge WebSocket server started on port 9090
```

### Step 3: Open Dashboard (Browser)

Open `simple_dashboard.html` in your browser:

```bash
# Option 1: Open directly
xdg-open /home/mazhar/final_stand/vrx/dashboard/simple_dashboard.html

# Option 2: Use Python simple server
cd /home/mazhar/final_stand/vrx/dashboard
python3 -m http.server 8000
# Then open: http://localhost:8000/simple_dashboard.html
```

## 🎯 Features

- ✅ **Real-time boat position** on map
- ✅ **Heading indicator** (blue arrow)
- ✅ **Waypoint marker** (red marker)
- ✅ **Telemetry sidebar**:
  - GPS coordinates
  - Heading
  - Speed
  - Distance to waypoint
  - Wind direction
- ✅ **Connection status** indicator

## 📡 ROS Topics Used

| Topic | Type | Description |
|-------|------|-------------|
| `/wamv/sensors/gps/gps/fix` | `sensor_msgs/NavSatFix` | GPS position |
| `/wamv/pose` | `geometry_msgs/Pose` | Boat pose (position + heading) |
| `/vrx/debug/wind/direction` | `std_msgs/Float32` | Wind direction |
| `/wamv/target_waypoint` | `geometry_msgs/Point` | Target waypoint (optional) |

## 🔧 Customization

### Change Map Center

Edit `simple_dashboard.html`, line ~120:
```javascript
const map = L.map('map').setView([-33.8568, 151.2153], 15);
//                                    ^lat      ^lon    ^zoom
```

### Set Waypoint Programmatically

In browser console:
```javascript
setWaypoint(-33.8573, 151.2158);
```

### Publish Waypoint from ROS

```bash
ros2 topic pub /wamv/target_waypoint geometry_msgs/msg/Point "{x: -33.8573, y: 151.2158, z: 0.0}"
```

## 🐛 Troubleshooting

**Problem**: "Disconnected" status
- **Fix**: Make sure rosbridge is running on port 9090
- Check: `netstat -tuln | grep 9090`

**Problem**: Boat marker not moving
- **Fix**: Verify GPS topic is publishing: `ros2 topic echo /wamv/sensors/gps/gps/fix`
- Check browser console for errors (F12)

**Problem**: Map not loading
- **Fix**: Check internet connection (Leaflet loads tiles from OpenStreetMap)
- Or use offline map tiles

## 📝 Next Steps

1. Add multiple waypoint support
2. Add path history trail
3. Add mission status indicators
4. Convert to React (see `react_dashboard/` folder)

