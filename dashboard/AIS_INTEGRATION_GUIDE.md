# 🚢 AIS Integration Guide

## Overview

This system integrates real-world AIS (Automatic Identification System) vessel data into both:
1. **Gazebo/ROS** - Vessels appear as 3D models in the simulation
2. **Dashboard** - Vessels appear as markers on the web map

## Architecture

```
AISStream.io (Real AIS Data)
    ↓
AIS Proxy (ais_proxy.py)
    ├──→ WebSocket (port 9091) → Dashboard (JavaScript)
    └──→ ROS Topics → Gazebo Visualization
```

## Components

### 1. **AIS Proxy** (`ais_proxy.py`)
- Connects to AISStream.io WebSocket API
- Forwards vessel data to:
  - Dashboard via WebSocket (port 9091)
  - ROS topics (via AIS ROS Bridge)

### 2. **AIS ROS Bridge** (`ais_ros_bridge.py`)
- Subscribes to AIS proxy WebSocket
- Publishes vessel positions to ROS topics:
  - `/ais/vessels` (PoseStamped)
  - `/ais/vessel_info` (String - JSON)

### 3. **AIS Gazebo Spawner** (`ais_gazebo_spawner.py`)
- Subscribes to `/ais/vessel_info`
- Spawns vessel models in Gazebo
- Updates vessel positions

### 4. **Dashboard Integration** (`dashboard.html`)
- Connects to AIS proxy WebSocket
- Displays vessels as red triangle markers
- Shows vessel info on click

## Setup

### Prerequisites

```bash
# Install Python dependencies
pip3 install websockets --user
```

### API Key

Your AISStream API key is already configured:
```
70e4762d6c570a3204ec48ddac3c08e1649320a5
```

## Usage

### Start AIS System

```bash
cd ~/final_stand/vrx/dashboard
./start_ais.sh
```

This will start:
1. AIS Proxy (connects to AISStream.io)
2. AIS ROS Bridge (publishes to ROS)
3. AIS Gazebo Spawner (spawns vessels in Gazebo)

### Verify It's Working

**1. Check AIS Proxy:**
```bash
tail -f /tmp/ais_proxy.log
```
Should see:
```
✅ Connected to AISStream.io
✅ Subscribed to AIS messages
📡 Tracking X vessels
```

**2. Check ROS Topics:**
```bash
source /opt/ros/jazzy/setup.bash
ros2 topic echo /ais/vessel_info
```
Should see JSON with vessel data.

**3. Check Dashboard:**
- Open `http://localhost:8000/dashboard.html`
- Open browser console (F12)
- Should see: `✅ AIS WebSocket connected`
- Red triangle markers should appear on map

**4. Check Gazebo:**
- Vessels should appear as red box models in Gazebo
- They will spawn at positions matching real AIS coordinates

## Configuration

### Change Bounding Box (Area to Monitor)

Edit `ais_proxy.py`, find:
```python
subscribe_message = {
    "APIKey": AISSTREAM_API_KEY,
    "BoundingBoxes": [[
        [-34.0, 150.0],  # Southwest corner
        [-33.0, 151.5]   # Northeast corner
    ]]
}
```

**Example: Singapore (more ships):**
```python
"BoundingBoxes": [[
    [1.0, 103.5],   # Southwest
    [1.5, 104.5]    # Northeast
]]
```

### Change Vessel Model in Gazebo

Edit `ais_gazebo_spawner.py`, modify the `vessel_sdf` string to use a different model.

**Example: Use a boat model from Fuel:**
```python
vessel_sdf = f'''<?xml version="1.0"?>
<sdf version="1.9">
  <include>
    <uri>https://fuel.gazebosim.org/1.0/openrobotics/models/simple_boat</uri>
    <pose>{x} {y} 0 0 0 {heading_rad}</pose>
  </include>
</sdf>'''
```

## Troubleshooting

### No Vessels Appearing

**Problem:** No ships in bounding box

**Solution:**
1. Widen the bounding box
2. Use a busier area (Singapore, English Channel, etc.)
3. Check AIS proxy logs for connection status

### Dashboard Not Showing Vessels

**Problem:** WebSocket connection failed

**Solution:**
1. Check AIS proxy is running: `ps aux | grep ais_proxy`
2. Check port 9091: `lsof -i :9091`
3. Open browser console, check for WebSocket errors
4. Restart AIS proxy: `./start_ais.sh`

### Gazebo Not Spawning Vessels

**Problem:** Gazebo service not available

**Solution:**
1. Make sure Gazebo/VRX is running first
2. Check service exists: `ros2 service list | grep create`
3. Check spawner logs: `tail -f /tmp/ais_gazebo_spawner.log`

### API Key Error

**Problem:** Invalid API key

**Solution:**
1. Verify API key in `ais_proxy.py`
2. Check AISStream.io account status
3. Test connection manually:
   ```bash
   wscat -c "wss://stream.aisstream.io/v0/stream?apiKey=YOUR_KEY"
   ```

## Integration with Full System

To start everything together:

```bash
# Terminal 1: Start VRX + Dashboard
cd ~/final_stand/vrx/dashboard
./LAUNCH_FULL_SYSTEM.sh

# Terminal 2: Start AIS
cd ~/final_stand/vrx/dashboard
./start_ais.sh
```

## Files

- `ais_proxy.py` - AIS proxy server
- `ais_ros_bridge.py` - ROS bridge for AIS data
- `ais_gazebo_spawner.py` - Gazebo vessel spawner
- `start_ais.sh` - Launch script
- `dashboard.html` - Updated with AIS visualization

## API Reference

### ROS Topics

- `/ais/vessels` (geometry_msgs/PoseStamped) - Vessel positions
- `/ais/vessel_info` (std_msgs/String) - JSON vessel data

### WebSocket

- `ws://localhost:9091` - AIS proxy WebSocket
- Message format:
  ```json
  {
    "type": "vessel_update",
    "mmsi": "123456789",
    "data": {
      "mmsi": "123456789",
      "name": "Ship Name",
      "latitude": -33.722,
      "longitude": 150.674,
      "heading": 45.0,
      "speed": 12.5,
      "course": 45.0
    }
  }
  ```

## Notes

- Vessels update in real-time (as fast as AISStream provides data)
- Old vessels are kept in memory (not removed automatically)
- Gazebo spawns vessels once, doesn't update positions (would need separate service)
- Dashboard updates vessel positions continuously

