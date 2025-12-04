# 🗺️ GPS Data Flow: Gazebo → Dashboard (Real-Time)

## Complete Architecture Overview

```
┌─────────────┐      ┌──────────────┐      ┌─────────────┐      ┌──────────────┐      ┌─────────────┐
│   Gazebo    │ ───> │ Gazebo-ROS   │ ───> │ ROS Topic   │ ───> │ rosbridge   │ ───> │ Dashboard  │
│  Simulation │      │    Bridge     │      │  (ROS 2)    │      │   Server    │      │  (Browser) │
└─────────────┘      └──────────────┘      └─────────────┘      └──────────────┘      └─────────────┘
     GPS Sensor         Message Convert      /wamv/sensors/      WebSocket (9090)      ROSLIB.js
```

---

## Step-by-Step Data Flow

### **Step 1: Gazebo GPS Sensor** 📡

**Location:** Gazebo simulation world (`sydney_regatta.sdf` or model definition)

The WAM-V boat model includes a GPS sensor plugin that:
- Reads the boat's position in Gazebo's world coordinates (x, y, z)
- Converts world coordinates to GPS coordinates (latitude, longitude, altitude)
- Publishes GPS data on Gazebo's internal topic: `/model/wamv/sensors/gps/gps/fix`

**Gazebo Topic Format:**
```
Topic: /model/wamv/sensors/gps/gps/fix
Type: gz.msgs.NavSatFix
```

**Example Gazebo Message:**
```json
{
  "header": {
    "stamp": {"sec": 1234567890, "nsec": 123456789},
    "frame_id": "gps"
  },
  "latitude": -33.724223,
  "longitude": 150.679736,
  "altitude": 0.0,
  "position_covariance": [...],
  "position_covariance_type": 0
}
```

---

### **Step 2: Gazebo-ROS Bridge** 🌉

**Location:** VRX Gazebo bridge configuration (typically in `vrx_gz` package)

The Gazebo-ROS bridge is a ROS 2 node that:
- Subscribes to Gazebo topics (like `/model/wamv/sensors/gps/gps/fix`)
- Converts Gazebo message types to ROS 2 message types
- Publishes on ROS 2 topics

**Bridge Configuration Example:**
```python
# In vrx_gz/src/vrx_gz/bridges.py or launch file
def gps(world_name, model_name):
    return Bridge(
        gz_topic=f'/model/{model_name}/sensors/gps/gps/fix',
        ros_topic=f'/{model_name}/sensors/gps/gps/fix',
        gz_type='gz.msgs.NavSatFix',
        ros_type='sensor_msgs/msg/NavSatFix',
        direction=BridgeDirection.GZ_TO_ROS
    )
```

**What Happens:**
1. Bridge subscribes to: `/model/wamv/sensors/gps/gps/fix` (Gazebo)
2. Bridge converts: `gz.msgs.NavSatFix` → `sensor_msgs/NavSatFix`
3. Bridge publishes to: `/wamv/sensors/gps/gps/fix` (ROS 2)

**ROS 2 Topic:**
```
Topic: /wamv/sensors/gps/gps/fix
Type: sensor_msgs/msg/NavSatFix
```

---

### **Step 3: ROS 2 Topic** 📢

**Location:** ROS 2 topic namespace `/wamv/sensors/gps/gps/fix`

The GPS data is now available as a standard ROS 2 topic that any ROS node can subscribe to.

**Message Structure (sensor_msgs/NavSatFix):**
```python
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
float64 latitude      # Degrees (WGS84)
float64 longitude     # Degrees (WGS84)
float64 altitude      # Meters (WGS84)
float64[9] position_covariance
uint8 position_covariance_type
```

**Verify Topic Exists:**
```bash
# List all topics
ros2 topic list | grep gps

# Echo GPS data
ros2 topic echo /wamv/sensors/gps/gps/fix

# Check topic info
ros2 topic info /wamv/sensors/gps/gps/fix
```

**Example Output:**
```bash
$ ros2 topic echo /wamv/sensors/gps/gps/fix
---
header:
  stamp:
    sec: 1234567890
    nanosec: 123456789
  frame_id: gps
latitude: -33.724223
longitude: 150.679736
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
1. rosbridge_server subscribes to ROS topics (like `/wamv/sensors/gps/gps/fix`)
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

**Message Format (JSON over WebSocket):**
```json
{
  "op": "publish",
  "topic": "/wamv/sensors/gps/gps/fix",
  "msg": {
    "header": {
      "stamp": {"sec": 1234567890, "nanosec": 123456789},
      "frame_id": "gps"
    },
    "latitude": -33.724223,
    "longitude": 150.679736,
    "altitude": 0.0,
    "position_covariance": [0, 0, 0, 0, 0, 0, 0, 0, 0],
    "position_covariance_type": 0
  }
}
```

---

### **Step 5: Dashboard JavaScript (ROSLIB.js)** 🖥️

**Location:** `dashboard/dashboard.html` (lines 736-768)

**Library:** ROSLIB.js (JavaScript library for ROS communication)

**Code Flow:**

#### **5.1. Connect to rosbridge:**
```javascript
// Line 638-640
ros = new ROSLIB.Ros({
    url: 'ws://localhost:9090'  // WebSocket connection to rosbridge
});
```

#### **5.2. Create Topic Subscription:**
```javascript
// Lines 737-741
const gpsTopic = new ROSLIB.Topic({
    ros: ros,                                    // ROS connection
    name: '/wamv/sensors/gps/gps/fix',          // ROS topic name
    messageType: 'sensor_msgs/NavSatFix'         // Message type
});
```

#### **5.3. Subscribe and Handle Messages:**
```javascript
// Lines 743-764
gpsTopic.subscribe((message) => {
    try {
        // Update topic status
        topicStatus['/wamv/sensors/gps/gps/fix'].lastMessage = Date.now();
        topicStatus['/wamv/sensors/gps/gps/fix'].messageCount++;
        
        // Validate GPS data
        if (message && message.latitude !== undefined && message.longitude !== undefined) {
            if (message.latitude !== 0 && message.longitude !== 0) {
                // Update boat state
                boatState.position = {
                    lat: message.latitude,
                    lon: message.longitude,
                    alt: message.altitude || 0
                };
                
                // Update map marker position
                updateBoatPosition();
                
                // Update telemetry display
                updatePositionTelemetry();
            } else {
                console.warn('⚠️ GPS data has zero coordinates, ignoring');
            }
        }
    } catch (error) {
        console.error('❌ Error processing GPS message:', error);
    }
});
```

#### **5.4. Update Map Marker:**
```javascript
// Lines 1087-1114
function updateBoatPosition() {
    const { lat, lon } = boatState.position;
    
    // Update marker position on map
    boatMarker.setLatLng([lat, lon]);
    
    // Add to path trail
    pathTrail.push([lat, lon]);
    
    // Limit trail length (keep last 1000 points)
    if (pathTrail.length > 1000) {
        pathTrail.shift();
    }
    
    // Update path polyline on map
    if (pathPolyline) {
        pathPolyline.setLatLngs(pathTrail);
    } else {
        pathPolyline = L.polyline(pathTrail, {
            color: '#4CAF50',
            weight: 3,
            opacity: 0.6
        }).addTo(mainMap);
    }
}
```

---

## Complete Code Reference

### **Dashboard HTML (dashboard.html):**

```javascript
// ============================================
// ROS CONNECTION
// ============================================
function connectROS() {
    ros = new ROSLIB.Ros({
        url: 'ws://localhost:9090'  // Connect to rosbridge
    });
    
    ros.on('connection', () => {
        console.log('✅ Connected to ROS!');
        subscribeToTopics();
    });
}

// ============================================
// GPS TOPIC SUBSCRIPTION
// ============================================
function subscribeToTopics() {
    // GPS Position Topic
    const gpsTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/wamv/sensors/gps/gps/fix',
        messageType: 'sensor_msgs/NavSatFix'
    });
    
    gpsTopic.subscribe((message) => {
        // Extract GPS coordinates
        const lat = message.latitude;
        const lon = message.longitude;
        const alt = message.altitude || 0;
        
        // Update boat state
        boatState.position = { lat, lon, alt };
        
        // Update map marker
        boatMarker.setLatLng([lat, lon]);
        
        // Update telemetry display
        document.getElementById('lat-value').textContent = lat.toFixed(6);
        document.getElementById('lon-value').textContent = lon.toFixed(6);
        document.getElementById('altitude-value').textContent = alt.toFixed(2);
    });
}
```

---

## Real-Time Update Frequency

**Typical Update Rates:**
- **Gazebo GPS Sensor:** 10-20 Hz (50-100ms intervals)
- **Gazebo-ROS Bridge:** Same as sensor (no buffering)
- **ROS Topic:** Same as bridge (direct passthrough)
- **rosbridge_server:** Same as ROS topic (real-time forwarding)
- **Dashboard:** Updates immediately when message arrives (no polling)

**Total Latency:** Typically < 100ms from Gazebo to dashboard display

---

## Verification Commands

### **1. Check Gazebo Topic:**
```bash
gz topic -l | grep gps
gz topic -e /model/wamv/sensors/gps/gps/fix
```

### **2. Check ROS Topic:**
```bash
ros2 topic list | grep gps
ros2 topic echo /wamv/sensors/gps/gps/fix
ros2 topic hz /wamv/sensors/gps/gps/fix
```

### **3. Check rosbridge:**
```bash
# Check if rosbridge is running
ps aux | grep rosbridge

# Check WebSocket connection (in browser console)
# Should see: "✅ Connected to ROS!"
```

### **4. Check Dashboard:**
```javascript
// In browser console (F12)
// Should see GPS messages logged:
console.log('🚤 Updating boat position:', lat, lon);
```

---

## Troubleshooting

### **Problem: No GPS data on dashboard**

**Check 1: Gazebo sensor working?**
```bash
gz topic -e /model/wamv/sensors/gps/gps/fix
# Should see messages with lat/lon
```

**Check 2: Bridge working?**
```bash
ros2 topic echo /wamv/sensors/gps/gps/fix
# Should see ROS messages
```

**Check 3: rosbridge running?**
```bash
ros2 node list | grep rosbridge
# Should see rosbridge_server node
```

**Check 4: Dashboard connected?**
```javascript
// Browser console (F12)
console.log(rosConnected);  // Should be true
console.log(ros);           // Should be ROSLIB.Ros object
```

---

## Summary

**Data Flow Chain:**
1. **Gazebo** → GPS sensor publishes `gz.msgs.NavSatFix` on `/model/wamv/sensors/gps/gps/fix`
2. **Bridge** → Converts to `sensor_msgs/NavSatFix` and publishes on `/wamv/sensors/gps/gps/fix`
3. **rosbridge** → Subscribes to ROS topic and forwards via WebSocket (port 9090)
4. **ROSLIB.js** → Connects to WebSocket and subscribes to topic
5. **Dashboard** → Receives messages, updates `boatMarker.setLatLng([lat, lon])` on Leaflet map

**Key Files:**
- Gazebo sensor: Model definition (SDF/URDF)
- Bridge: `vrx_gz/src/vrx_gz/bridges.py` or launch file
- ROS topic: `/wamv/sensors/gps/gps/fix`
- rosbridge: `ros2 run rosbridge_server rosbridge_websocket`
- Dashboard: `dashboard/dashboard.html` (lines 736-768, 1087-1114)

**Real-Time:** Yes! Updates flow continuously as boat moves in Gazebo.

