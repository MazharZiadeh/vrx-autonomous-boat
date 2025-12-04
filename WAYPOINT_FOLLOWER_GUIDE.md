# 🧭 GPS Waypoint Follower Guide

## 📁 File Structure

```
vrx_ws/
└── src/
    └── wamv_autonomy/
        ├── package.xml
        ├── setup.py
        └── wamv_autonomy/
            ├── __init__.py
            ├── waypoint_follower.py    ← NEW: GPS waypoint follower
            └── autonomy_node.py        ← Original autonomy node
```

## 🔧 Build the Package

```bash
cd /home/mazhar/vrx_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select wamv_autonomy
source install/setup.bash
```

## 🎯 How It Works

1. **Reads Boat Pose**: Subscribes to `/wamv/pose` (position + orientation)
2. **Reads GPS**: Subscribes to `/wamv/sensors/gps/gps/fix` (latitude, longitude)
3. **Calculates Heading Error**: Uses Haversine formula to compute bearing to target
4. **Publishes Thrust**: Sends commands to `/wamv/thrusters/left/thrust` and `/wamv/thrusters/right/thrust`

## 🚀 Usage

### Step 1: Set Target GPS Coordinates

Edit the target waypoint in `waypoint_follower.py`:

```python
# Line 18-19: Change these to your target location
self.target_lat = -33.8568  # Your target latitude
self.target_lon = 151.2153  # Your target longitude
```

### Step 2: Launch the Node

```bash
cd /home/mazhar/vrx_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run wamv_autonomy waypoint_follower
```

### Step 3: Watch It Work

The node will:
- Wait for GPS fix
- Set origin on first GPS reading
- Calculate distance and bearing to target
- Publish thrust commands to move boat
- Log status every second

## 📊 Expected Output

```
[INFO] [waypoint_follower]: Waypoint Follower Node started
[INFO] [waypoint_follower]: Target waypoint: (-33.8568, 151.2153)
[INFO] [waypoint_follower]: Waiting for GPS fix...
[INFO] [waypoint_follower]: Origin set: (-33.8568, 151.2153)
[INFO] [waypoint_follower]: GPS: (-33.8568, 151.2153) | Distance: 150.5m | Bearing: 45.2° | Heading: 42.1° | Error: 3.1°
```

## 🧪 Testing

### Test 1: Check Current GPS

```bash
ros2 topic echo /wamv/sensors/gps/gps/fix --once
```

### Test 2: Set a Close Waypoint

For testing, set a waypoint 50-100 meters away:

```python
# In waypoint_follower.py, modify:
self.target_lat = <current_lat + 0.0005>  # ~55 meters north
self.target_lon = <current_lon + 0.0005>  # ~55 meters east
```

### Test 3: Monitor Thrust Commands

```bash
ros2 topic echo /wamv/thrusters/left/thrust
ros2 topic echo /wamv/thrusters/right/thrust
```

## ⚙️ Tuning Parameters

In `waypoint_follower.py`, adjust these for better performance:

```python
self.max_thrust = 800.0        # Maximum thrust (0-1000)
self.kp_heading = 3.0         # Heading correction gain (higher = faster turning)
self.kp_distance = 0.8        # Distance gain (higher = faster approach)
self.waypoint_tolerance = 10.0 # Consider waypoint reached at this distance (meters)
```

## 🐛 Troubleshooting

**Problem**: Node says "Waiting for GPS fix..."
- **Fix**: Make sure VRX is running and boat is spawned
- Check: `ros2 topic echo /wamv/sensors/gps/gps/fix`

**Problem**: Boat spins in circles
- **Fix**: Reduce `kp_heading` (try 1.5 instead of 3.0)

**Problem**: Boat moves too slow
- **Fix**: Increase `max_thrust` or `kp_distance`

**Problem**: Boat overshoots waypoint
- **Fix**: Increase `waypoint_tolerance` or reduce `kp_distance`

## 📝 Code Overview

### Key Functions

- `gps_callback()`: Updates GPS position, sets origin
- `pose_callback()`: Updates boat pose (position + heading)
- `calculate_distance()`: Haversine formula for GPS distance
- `calculate_bearing()`: Computes heading to target
- `control_loop()`: Main control logic (runs at 10 Hz)
- `publish_thrust()`: Sends commands to boat thrusters

### Control Algorithm

1. Calculate distance to target (Haversine)
2. Calculate bearing to target (atan2)
3. Get current heading from pose
4. Compute heading error
5. Apply proportional control:
   - Base thrust = distance × kp_distance
   - Heading correction = heading_error × kp_heading
   - Left thrust = base - correction
   - Right thrust = base + correction

