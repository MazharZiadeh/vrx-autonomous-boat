# 🚤 NPC Boats GPS Data Flow: Gazebo → Dashboard (Real-Time)

## Complete Architecture Overview

```
┌─────────────┐      ┌──────────────────┐      ┌─────────────┐      ┌──────────────┐      ┌─────────────┐
│   Gazebo    │ ───> │ spawned_boats_   │ ───> │ ROS Topic   │ ───> │ rosbridge   │ ───> │ Dashboard  │
│  Simulation │      │ publisher.py     │      │  (ROS 2)    │      │   Server    │      │  (Browser) │
│ (wamv_boat_ │      │ (Service Query)  │      │ /wamv_boat_ │      │ WebSocket   │      │ ROSLIB.js  │
│   1-20)     │      │                  │      │ X/sensors/  │      │  (9090)     │      │            │
└─────────────┘      └──────────────────┘      └─────────────┘      └──────────────┘      └─────────────┘
  Model Poses          Pose → GPS Convert        gps/gps/fix         JSON Messages      Red Markers
```

---

## Step-by-Step Data Flow

### **Step 1: Gazebo Model Poses** 📡

**Location:** Gazebo simulation world (`sydney_regatta.sdf`)

The spawned NPC boats (`wamv_boat_1` through `wamv_boat_20`) exist as models in Gazebo:
- Each boat has a position (x, y, z) in Gazebo world coordinates
- Each boat has an orientation (quaternion) representing heading
- **IMPORTANT:** NPC boats do NOT have GPS sensors like the main boat
- Their positions are stored in Gazebo's entity component manager (ECM)

**Gazebo Entity Names:**
```
wamv_boat_1
wamv_boat_2
wamv_boat_3
...
wamv_boat_20
```

**Gazebo Coordinates:**
- Origin: Sydney Regatta area
- X-axis: East-West (meters)
- Y-axis: North-South (meters)
- Z-axis: Up-Down (meters, 0 = water surface)

**Example Gazebo Pose:**
```
Entity: wamv_boat_1
Position: x = -500.0, y = 150.0, z = 0.0
Orientation: qx = 0.0, qy = 0.0, qz = 0.707, qw = 0.707 (45° heading)
```

---

### **Step 2: spawned_boats_publisher.py** 🌉

**Location:** `dashboard/spawned_boats_publisher.py`

**Purpose:** This ROS 2 node queries Gazebo for each NPC boat's pose and converts it to GPS coordinates.

**How it Works:**

#### **2.1. Service Client Setup:**
```python
# Lines 26-40
self.world_name = 'simple_demo'  # Gazebo world name
self.pose_client = self.create_client(
    EntityPose,
    f'/world/{self.world_name}/pose'  # Gazebo pose service
)
```

**Service Details:**
- **Service Name:** `/world/simple_demo/pose` (or `/world/sydney_regatta/pose`)
- **Service Type:** `ros_gz_interfaces/srv/EntityPose`
- **Function:** Queries Gazebo for a specific entity's pose

#### **2.2. Periodic Query (1 Hz):**
```python
# Lines 42-43
# Timer to periodically query boat poses (1 Hz)
self.create_timer(1.0, self.update_boat_positions)
```

**What Happens:**
- Every 1 second, the node queries all 20 boats
- For each boat (`wamv_boat_1` to `wamv_boat_20`), it calls the pose service

#### **2.3. Query Individual Boat:**
```python
# Lines 45-56
def query_boat_pose(self, boat_name):
    """Query a single boat's pose and publish GPS"""
    try:
        # Create request
        request = EntityPose.Request()
        request.entity.name = boat_name  # e.g., "wamv_boat_1"
        
        # Call service (async)
        future = self.pose_client.call_async(request)
        
        # Process result when ready
        future.add_done_callback(lambda f, name=boat_name: self.pose_callback(f, name))
    except Exception as e:
        pass  # Silently ignore errors (boat might not exist)
```

#### **2.4. Convert Pose to GPS:**
```python
# Lines 58-95
def pose_callback(self, future, boat_name):
    """Process pose service response"""
    try:
        response = future.result()
        if response and response.pose:
            pose = response.pose
            
            # Convert Gazebo coordinates to lat/lon
            x = pose.position.x  # Gazebo X (East-West in meters)
            y = pose.position.y  # Gazebo Y (North-South in meters)
            
            # Sydney Regatta origin
            SYDNEY_ORIGIN_LAT = -33.724223
            SYDNEY_ORIGIN_LON = 150.679736
            METERS_PER_DEG_LAT = 111000.0
            METERS_PER_DEG_LON = 111000.0 * math.cos(math.radians(SYDNEY_ORIGIN_LAT))
            
            # Convert to GPS coordinates
            lat = SYDNEY_ORIGIN_LAT + (y / METERS_PER_DEG_LAT)
            lon = SYDNEY_ORIGIN_LON + (x / METERS_PER_DEG_LON)
```

**Coordinate Conversion Formula:**
```
latitude  = SYDNEY_ORIGIN_LAT + (y_meters / 111000.0)
longitude = SYDNEY_ORIGIN_LON + (x_meters / (111000.0 * cos(latitude)))
```

**Example Conversion:**
```
Gazebo: x = -500.0, y = 150.0
→ lat = -33.724223 + (150.0 / 111000.0) = -33.722872
→ lon = 150.679736 + (-500.0 / (111000.0 * cos(-33.724223))) = 150.675234
```

#### **2.5. Publish GPS Topic:**
```python
# Lines 75-95
# Create publisher if it doesn't exist
if boat_name not in self.boat_publishers:
    topic_name = f'/{boat_name}/sensors/gps/gps/fix'
    pub = self.create_publisher(NavSatFix, topic_name, 10)
    self.boat_publishers[boat_name] = pub
    self.get_logger().info(f'   Created GPS publisher for {boat_name} at {topic_name}')

# Publish GPS fix
gps_msg = NavSatFix()
gps_msg.header.stamp = self.get_clock().now().to_msg()
gps_msg.header.frame_id = 'gps'
gps_msg.latitude = lat
gps_msg.longitude = lon
gps_msg.altitude = 0.0
gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

self.boat_publishers[boat_name].publish(gps_msg)
```

**Published Topics:**
```
/wamv_boat_1/sensors/gps/gps/fix
/wamv_boat_2/sensors/gps/gps/fix
/wamv_boat_3/sensors/gps/gps/fix
...
/wamv_boat_20/sensors/gps/gps/fix
```

**Message Type:** `sensor_msgs/msg/NavSatFix` (same as main boat!)

---

### **Step 3: ROS 2 Topics** 📢

**Location:** ROS 2 topic namespace `/wamv_boat_X/sensors/gps/gps/fix`

Each NPC boat now has its own GPS topic, identical in structure to the main boat's GPS topic.

**Topic Structure:**
```
Topic: /wamv_boat_1/sensors/gps/gps/fix
Type: sensor_msgs/msg/NavSatFix
Publisher: spawned_boats_publisher node
Update Rate: 1 Hz (1 update per second per boat)
```

**Message Structure (sensor_msgs/NavSatFix):**
```python
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
float64 latitude      # Degrees (WGS84)
float64 longitude     # Degrees (WGS84)
float64 altitude      # Always 0.0 (water surface)
float64[9] position_covariance
uint8 position_covariance_type
```

**Verify Topics Exist:**
```bash
# List all NPC boat GPS topics
ros2 topic list | grep "wamv_boat.*gps"

# Echo GPS data for boat 1
ros2 topic echo /wamv_boat_1/sensors/gps/gps/fix

# Check update rate
ros2 topic hz /wamv_boat_1/sensors/gps/gps/fix
# Should show ~1.0 Hz
```

**Example Output:**
```bash
$ ros2 topic echo /wamv_boat_1/sensors/gps/gps/fix
---
header:
  stamp:
    sec: 1234567890
    nanosec: 123456789
  frame_id: gps
latitude: -33.722872
longitude: 150.675234
altitude: 0.0
position_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
position_covariance_type: 0
---
```

---

### **Step 4: rosbridge_server** 🌐

**Location:** Running as ROS 2 node (typically launched with `ros2 run rosbridge_server rosbridge_websocket`)

**Purpose:** Exposes ROS topics via WebSocket so web browsers can access them.

**How it Works:**
1. rosbridge_server subscribes to ALL ROS topics (including NPC boat GPS topics)
2. When a message arrives, it serializes it to JSON
3. Sends JSON over WebSocket to connected clients (port 9090)

**Launch Command:**
```bash
ros2 run rosbridge_server rosbridge_websocket --ros-args -p port:=9090
```

**WebSocket Connection:**
```
ws://localhost:9090
```

**Message Format (JSON over WebSocket) for NPC Boat:**
```json
{
  "op": "publish",
  "topic": "/wamv_boat_1/sensors/gps/gps/fix",
  "msg": {
    "header": {
      "stamp": {"sec": 1234567890, "nanosec": 123456789},
      "frame_id": "gps"
    },
    "latitude": -33.722872,
    "longitude": 150.675234,
    "altitude": 0.0,
    "position_covariance": [0, 0, 0, 0, 0, 0, 0, 0, 0],
    "position_covariance_type": 0
  }
}
```

---

### **Step 5: Dashboard JavaScript (ROSLIB.js)** 🖥️

**Location:** `dashboard/dashboard.html` (lines 1080-1160)

**Library:** ROSLIB.js (JavaScript library for ROS communication)

**Code Flow:**

#### **5.1. Connect to rosbridge:**
```javascript
// Line 685-686 (same as main boat)
ros = new ROSLIB.Ros({
    url: 'ws://localhost:9090'  // WebSocket connection to rosbridge
});
```

#### **5.2. Subscribe to All NPC Boats:**
```javascript
// Lines 1080-1120
function subscribeToSpawnedBoats() {
    if (!ros || !rosConnected) {
        console.warn('⚠️ Cannot subscribe to spawned boats: ROS not connected');
        return;
    }
    
    console.log('🚤 Subscribing to spawned boats (wamv_boat_1 to wamv_boat_20)...');
    
    // Subscribe to GPS topics for each spawned boat
    for (let i = 1; i <= 20; i++) {
        const boatName = `wamv_boat_${i}`;
        
        // Subscribe to GPS fix topic (same structure as main boat)
        const gpsTopic = new ROSLIB.Topic({
            ros: ros,
            name: `/${boatName}/sensors/gps/gps/fix`,
            messageType: 'sensor_msgs/NavSatFix'
        });
```

#### **5.3. Handle GPS Messages:**
```javascript
// Lines 1100-1120
gpsTopic.subscribe((message) => {
    try {
        if (message && message.latitude !== undefined && message.longitude !== undefined) {
            if (message.latitude !== 0 && message.longitude !== 0) {
                const lat = message.latitude;
                const lon = message.longitude;
                
                // Get heading from IMU if available, otherwise use 0
                let heading = 0;
                
                // Update spawned boat marker on map
                updateSpawnedBoat(boatName, lat, lon, heading);
            }
        }
    } catch (error) {
        // Silently ignore errors (boat might not exist yet)
    }
});
```

#### **5.4. Update Red Boat Marker:**
```javascript
// Lines 1122-1160
function updateSpawnedBoat(boatName, lat, lon, heading) {
    // Skip if coordinates are invalid
    if (!lat || !lon || isNaN(lat) || isNaN(lon)) {
        return;
    }
    
    if (!spawnedBoats.has(boatName)) {
        // Create new RED boat marker (different from main boat's green)
        const redBoatIcon = L.divIcon({
            className: 'boat-marker-container',
            html: `<div class="boat-marker-red" id="boat-heading-${boatName}" style="transform: rotate(${heading - 90}deg);"></div>`,
            iconSize: [30, 30],
            iconAnchor: [15, 15]
        });
        
        const marker = L.marker([lat, lon], {
            icon: redBoatIcon
        }).addTo(mainMap);
        
        marker.bindPopup(`<b>${boatName}</b><br>Non-main boat`);
        
        spawnedBoats.set(boatName, {
            marker: marker,
            heading: heading
        });
        
        console.log(`🚤 Added spawned boat: ${boatName} at ${lat.toFixed(6)}, ${lon.toFixed(6)}`);
    } else {
        // Update existing boat position
        const boat = spawnedBoats.get(boatName);
        boat.marker.setLatLng([lat, lon]);
        
        // Update heading rotation
        const indicator = document.getElementById(`boat-heading-${boatName}`);
        if (indicator) {
            indicator.style.transform = `rotate(${heading - 90}deg)`;
        }
        
        boat.heading = heading;
    }
}
```

**Key Differences from Main Boat:**
- **Color:** Red markers (`boat-marker-red`) vs green (`boat-marker`) for main boat
- **Topic Names:** `/wamv_boat_X/sensors/gps/gps/fix` vs `/wamv/sensors/gps/gps/fix`
- **Update Rate:** 1 Hz (queried) vs 10-20 Hz (sensor-driven) for main boat
- **Data Source:** Service query → GPS conversion vs direct GPS sensor

---

## Complete Code Reference

### **spawned_boats_publisher.py:**

```python
#!/usr/bin/env python3
"""
ROS 2 node that publishes GPS coordinates for all spawned WAM-V boats.
Uses service calls to query model poses from Gazebo.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from ros_gz_interfaces.srv import EntityPose
import math

# Sydney Regatta origin (same as main boat)
SYDNEY_ORIGIN_LAT = -33.724223
SYDNEY_ORIGIN_LON = 150.679736
METERS_PER_DEG_LAT = 111000.0
METERS_PER_DEG_LON = 111000.0 * math.cos(math.radians(SYDNEY_ORIGIN_LAT))

class SpawnedBoatsPublisher(Node):
    def __init__(self):
        super().__init__('spawned_boats_publisher')
        
        # Service client for getting entity poses
        self.world_name = 'simple_demo'
        self.pose_client = self.create_client(
            EntityPose,
            f'/world/{self.world_name}/pose'
        )
        
        # Timer to periodically query boat poses (1 Hz)
        self.create_timer(1.0, self.update_boat_positions)
        
        self.boat_publishers = {}
    
    def update_boat_positions(self):
        """Periodically query and publish GPS for all spawned boats"""
        for i in range(1, 21):
            boat_name = f'wamv_boat_{i}'
            self.query_boat_pose(boat_name)
    
    def query_boat_pose(self, boat_name):
        """Query a single boat's pose and publish GPS"""
        request = EntityPose.Request()
        request.entity.name = boat_name
        future = self.pose_client.call_async(request)
        future.add_done_callback(lambda f, name=boat_name: self.pose_callback(f, name))
    
    def pose_callback(self, future, boat_name):
        """Process pose service response and publish GPS"""
        try:
            response = future.result()
            if response and response.pose:
                pose = response.pose
                
                # Convert Gazebo coordinates to lat/lon
                x = pose.position.x
                y = pose.position.y
                lat = SYDNEY_ORIGIN_LAT + (y / METERS_PER_DEG_LAT)
                lon = SYDNEY_ORIGIN_LON + (x / METERS_PER_DEG_LON)
                
                # Create publisher if needed
                if boat_name not in self.boat_publishers:
                    topic_name = f'/{boat_name}/sensors/gps/gps/fix'
                    pub = self.create_publisher(NavSatFix, topic_name, 10)
                    self.boat_publishers[boat_name] = pub
                
                # Publish GPS fix
                gps_msg = NavSatFix()
                gps_msg.header.stamp = self.get_clock().now().to_msg()
                gps_msg.header.frame_id = 'gps'
                gps_msg.latitude = lat
                gps_msg.longitude = lon
                gps_msg.altitude = 0.0
                gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
                
                self.boat_publishers[boat_name].publish(gps_msg)
        except Exception as e:
            pass  # Boat might not exist
```

### **Dashboard HTML (dashboard.html):**

```javascript
// ============================================
// SPAWNED BOATS VISUALIZATION
// ============================================
function subscribeToSpawnedBoats() {
    if (!ros || !rosConnected) {
        return;
    }
    
    console.log('🚤 Subscribing to spawned boats (wamv_boat_1 to wamv_boat_20)...');
    
    // Subscribe to GPS topics for each spawned boat
    for (let i = 1; i <= 20; i++) {
        const boatName = `wamv_boat_${i}`;
        
        // Subscribe to GPS fix topic
        const gpsTopic = new ROSLIB.Topic({
            ros: ros,
            name: `/${boatName}/sensors/gps/gps/fix`,
            messageType: 'sensor_msgs/NavSatFix'
        });
        
        gpsTopic.subscribe((message) => {
            try {
                if (message && message.latitude !== undefined && message.longitude !== undefined) {
                    if (message.latitude !== 0 && message.longitude !== 0) {
                        const lat = message.latitude;
                        const lon = message.longitude;
                        let heading = 0;
                        
                        updateSpawnedBoat(boatName, lat, lon, heading);
                    }
                }
            } catch (error) {
                // Silently ignore errors
            }
        });
    }
}

function updateSpawnedBoat(boatName, lat, lon, heading) {
    if (!spawnedBoats.has(boatName)) {
        // Create new RED boat marker
        const redBoatIcon = L.divIcon({
            className: 'boat-marker-container',
            html: `<div class="boat-marker-red" id="boat-heading-${boatName}" style="transform: rotate(${heading - 90}deg);"></div>`,
            iconSize: [30, 30],
            iconAnchor: [15, 15]
        });
        
        const marker = L.marker([lat, lon], {
            icon: redBoatIcon
        }).addTo(mainMap);
        
        marker.bindPopup(`<b>${boatName}</b><br>Non-main boat`);
        
        spawnedBoats.set(boatName, {
            marker: marker,
            heading: heading
        });
    } else {
        // Update existing boat
        const boat = spawnedBoats.get(boatName);
        boat.marker.setLatLng([lat, lon]);
        
        const indicator = document.getElementById(`boat-heading-${boatName}`);
        if (indicator) {
            indicator.style.transform = `rotate(${heading - 90}deg)`;
        }
        boat.heading = heading;
    }
}
```

---

## Real-Time Update Frequency

**NPC Boats Update Rates:**
- **Gazebo Model Pose:** Continuous (updated every simulation step)
- **Service Query:** 1 Hz (queried once per second per boat)
- **GPS Publisher:** 1 Hz (publishes once per second per boat)
- **ROS Topic:** 1 Hz (same as publisher)
- **rosbridge_server:** Same as ROS topic (real-time forwarding)
- **Dashboard:** Updates immediately when message arrives (no polling)

**Total Latency:** Typically < 200ms from Gazebo pose query to dashboard display

**Note:** NPC boats update slower (1 Hz) than main boat (10-20 Hz) because:
- Main boat has a dedicated GPS sensor that publishes continuously
- NPC boats require service calls to query poses, which is more expensive

---

## Verification Commands

### **1. Check spawned_boats_publisher is Running:**
```bash
ros2 node list | grep spawned_boats
# Should see: /spawned_boats_publisher

# Check node info
ros2 node info /spawned_boats_publisher
```

### **2. Check ROS Topics:**
```bash
# List all NPC boat GPS topics
ros2 topic list | grep "wamv_boat.*gps"

# Echo GPS data for boat 1
ros2 topic echo /wamv_boat_1/sensors/gps/gps/fix

# Check update rate (should be ~1.0 Hz)
ros2 topic hz /wamv_boat_1/sensors/gps/gps/fix
```

### **3. Check Service Availability:**
```bash
# List services
ros2 service list | grep pose

# Should see:
# /world/simple_demo/pose
# or
# /world/sydney_regatta/pose
```

### **4. Check Dashboard:**
```javascript
// In browser console (F12)
// Should see spawned boats logged:
console.log('🚤 Added spawned boat: wamv_boat_1 at ...');

// Check if boats are in the map
console.log(spawnedBoats.size);  // Should be 1-20
```

---

## Troubleshooting

### **Problem: No NPC boats on dashboard**

**Check 1: spawned_boats_publisher running?**
```bash
ps aux | grep spawned_boats_publisher
# Should see Python process

# Check logs
cat /tmp/spawned_boats_publisher.log
```

**Check 2: Topics being published?**
```bash
ros2 topic list | grep "wamv_boat.*gps"
# Should see 20 topics

ros2 topic echo /wamv_boat_1/sensors/gps/gps/fix
# Should see GPS messages
```

**Check 3: Service available?**
```bash
ros2 service list | grep "/world.*pose"
# Should see pose service
```

**Check 4: Dashboard connected?**
```javascript
// Browser console (F12)
console.log(rosConnected);  // Should be true
console.log(spawnedBoats.size);  // Should be > 0
```

### **Problem: Boats not moving on dashboard**

**Cause:** Service query rate is 1 Hz, so updates appear slower than main boat.

**Solution:** Increase update rate in `spawned_boats_publisher.py`:
```python
# Change from 1.0 to 2.0 for 2 Hz updates
self.create_timer(0.5, self.update_boat_positions)  # 2 Hz
```

---

## Key Differences: Main Boat vs NPC Boats

| Aspect | Main Boat | NPC Boats |
|--------|-----------|-----------|
| **GPS Source** | Built-in GPS sensor | Service query + conversion |
| **Topic Name** | `/wamv/sensors/gps/gps/fix` | `/wamv_boat_X/sensors/gps/gps/fix` |
| **Update Rate** | 10-20 Hz (sensor-driven) | 1 Hz (query-driven) |
| **Bridge** | Gazebo-ROS bridge (automatic) | spawned_boats_publisher.py (manual) |
| **Marker Color** | Green (`boat-marker`) | Red (`boat-marker-red`) |
| **Data Flow** | Sensor → Bridge → Topic | Service → Convert → Topic |

---

## Summary

**Data Flow Chain for NPC Boats:**
1. **Gazebo** → Model poses stored in entity component manager
2. **spawned_boats_publisher.py** → Queries pose service, converts to GPS, publishes on `/wamv_boat_X/sensors/gps/gps/fix`
3. **rosbridge** → Subscribes to ROS topics and forwards via WebSocket (port 9090)
4. **ROSLIB.js** → Connects to WebSocket and subscribes to NPC boat GPS topics
5. **Dashboard** → Receives messages, updates red markers: `marker.setLatLng([lat, lon])` on Leaflet map

**Key Files:**
- Publisher: `dashboard/spawned_boats_publisher.py`
- Dashboard: `dashboard/dashboard.html` (lines 1080-1160)
- Spawn script: `dashboard/spawn_wamv_boats.sh` (auto-starts publisher)
- ROS topics: `/wamv_boat_1/sensors/gps/gps/fix` through `/wamv_boat_20/sensors/gps/gps/fix`
- rosbridge: `ros2 run rosbridge_server rosbridge_websocket`

**Real-Time:** Yes! Updates flow continuously (1 Hz) as boats move in Gazebo, displayed as **RED markers** on the dashboard.

