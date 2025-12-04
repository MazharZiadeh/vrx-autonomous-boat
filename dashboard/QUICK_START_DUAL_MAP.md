# 🚀 Quick Start: Dual Map Dashboard

## ✅ What's Included

- ✅ **Main Map (80% viewport)** - Detailed boat tracking
- ✅ **MiniMap (bottom-right)** - Global overview
- ✅ **Boat marker** - Rotating triangle showing heading
- ✅ **Path trail** - Shows where boat has been
- ✅ **Waypoint markers** - Numbered pins
- ✅ **Planned path** - Dashed orange line
- ✅ **Telemetry sidebar** - Real-time data
- ✅ **ROS WebSocket** - Real-time updates

## 🚀 3-Step Setup

### Step 1: Install & Start rosbridge (Terminal 1)

```bash
cd /home/mazhar/final_stand/vrx/dashboard
./rosbridge_setup.sh
```

**Expected output:**
```
✅ rosbridge_server installed
Starting rosbridge WebSocket server...
Server will be available at: ws://localhost:9090
```

### Step 2: Start Web Server (Terminal 2)

```bash
cd /home/mazhar/final_stand/vrx/dashboard
python3 -m http.server 8000
```

### Step 3: Open Dashboard (Browser)

Open: **http://localhost:8000/dashboard.html**

## 🎯 Features

### Main Map
- Boat marker (blue triangle) rotates with heading
- Path trail (green line) shows boat history
- Planned path (orange dashed line) shows mission
- Waypoint markers (red numbered circles)

### MiniMap
- Bottom-right corner
- Shows zoomed-out view
- Rectangle shows main map viewport
- Synchronized with main map

### Telemetry Sidebar
- **Speed** (knots)
- **Heading** (degrees)
- **Position** (lat/lon)
- **Wind** (direction & speed)
- **Waypoint progress** (current/total, distance, ETA)

## 📡 ROS Topics Connected

| Topic | Type | Purpose |
|-------|------|---------|
| `/wamv/sensors/gps/gps/fix` | `sensor_msgs/NavSatFix` | Boat position |
| `/wamv/sensors/imu/imu/data` | `sensor_msgs/Imu` | Heading |
| `/wamv/pose` | `geometry_msgs/Pose` | Speed calculation |
| `/vrx/debug/wind/direction` | `std_msgs/Float32` | Wind direction |
| `/vrx/debug/wind/speed` | `std_msgs/Float32` | Wind speed |

## 🧪 Test It

1. **Without ROS** (test UI):
   - Open dashboard
   - Should see map, minimap, test waypoints
   - Connection status: "Disconnected"

2. **With ROS** (real data):
   - Start VRX with boat spawned
   - Start rosbridge
   - Open dashboard
   - Connection status: "Connected"
   - Watch boat move on map!

## 🔧 Coordinate Transform Functions

The dashboard includes helper functions:

```javascript
// ROS local coordinates → GPS
rosToGPS(rosX, rosY, originLat, originLon)

// GPS → ROS local coordinates  
gpsToROS(lat, lon, originLat, originLon)

// Distance between two GPS points (Haversine)
haversineDistance(lat1, lon1, lat2, lon2)
```

## 📝 File Structure

```
dashboard/
├── dashboard.html          ← Main dashboard (single file, everything inline)
├── rosbridge_setup.sh      ← Install & start rosbridge
├── TEST_DASHBOARD.md       ← Testing instructions
└── QUICK_START_DUAL_MAP.md ← This file
```

## ✅ Ready!

The dashboard is a **single HTML file** with:
- All CSS inline
- All JavaScript inline
- No build step required
- Works with `file://` or simple HTTP server

Just open it in a browser and connect to rosbridge!

