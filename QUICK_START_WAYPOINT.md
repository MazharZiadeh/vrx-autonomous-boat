# 🚀 Quick Start: GPS Waypoint Follower

## ✅ What's Created

**File**: `/home/mazhar/vrx_ws/src/wamv_autonomy/wamv_autonomy/waypoint_follower.py`

This node:
- ✅ Reads boat pose from `/wamv/pose`
- ✅ Reads GPS from `/wamv/sensors/gps/gps/fix`
- ✅ Calculates heading error using Haversine formula
- ✅ Publishes thrust commands to move boat to target GPS coordinate

## 📋 Step-by-Step Usage

### Step 1: Get Current Boat GPS

```bash
cd /home/mazhar/vrx_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 topic echo /wamv/sensors/gps/gps/fix --once
```

**Note the latitude and longitude values!**

### Step 2: Set Target Waypoint

Edit the file:
```bash
nano /home/mazhar/vrx_ws/src/wamv_autonomy/wamv_autonomy/waypoint_follower.py
```

Change lines 18-19:
```python
self.target_lat = -33.8568  # Replace with your target latitude
self.target_lon = 151.2153  # Replace with your target longitude
```

**For testing, set a waypoint 50-100m away:**
- Add ~0.0005 to latitude = ~55 meters north
- Add ~0.0005 to longitude = ~55 meters east

### Step 3: Rebuild Package

```bash
cd /home/mazhar/vrx_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select wamv_autonomy
source install/setup.bash
```

### Step 4: Run Waypoint Follower

```bash
ros2 run wamv_autonomy waypoint_follower
```

### Step 5: Watch It Work!

You'll see output like:
```
[INFO] [waypoint_follower]: Waypoint Follower Node started
[INFO] [waypoint_follower]: Target waypoint: (-33.8568, 151.2153)
[INFO] [waypoint_follower]: Origin set: (-33.8568, 151.2153)
[INFO] [waypoint_follower]: GPS: (-33.8568, 151.2153) | Distance: 150.5m | Bearing: 45.2° | Heading: 42.1° | Error: 3.1°
```

The boat will start moving toward the target!

## 🧪 Test Example

**Current GPS**: `-33.8568, 151.2153`  
**Target GPS**: `-33.8573, 151.2158` (about 70m away)

```python
self.target_lat = -33.8573
self.target_lon = 151.2158
```

## 📊 Monitor Boat Movement

**Terminal 1**: Watch waypoint follower logs  
**Terminal 2**: Monitor thrust commands
```bash
ros2 topic echo /wamv/thrusters/left/thrust
```
**Terminal 3**: Watch boat pose
```bash
ros2 topic echo /wamv/pose
```

## ⚙️ Tune Performance

If boat behavior isn't good, adjust in `waypoint_follower.py`:

```python
self.max_thrust = 800.0        # Max speed (0-1000)
self.kp_heading = 3.0         # Turn speed (higher = faster turning)
self.kp_distance = 0.8        # Approach speed
self.waypoint_tolerance = 10.0 # Stop when within this distance (meters)
```

## 🎯 File Structure

```
vrx_ws/
└── src/
    └── wamv_autonomy/
        ├── setup.py                    ← Entry point registered
        └── wamv_autonomy/
            └── waypoint_follower.py    ← Your new node!
```

## ✅ Done!

The waypoint follower is ready to use. Just:
1. Set target GPS coordinates
2. Rebuild
3. Run it!

