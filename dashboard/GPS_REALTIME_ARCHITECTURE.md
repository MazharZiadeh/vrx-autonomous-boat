# 🗺️ Real-Time GPS Dashboard Architecture

## 📊 Complete Data Flow

```
┌─────────────────────────────────────────────────────────────────┐
│ 1. GAZEBO/VRX SIMULATION                                         │
│    - GPS sensor on boat publishes NavSat messages               │
│    - Topic: Gazebo internal (gz.msgs.NavSat)                     │
└───────────────────────┬─────────────────────────────────────────┘
                        │
                        ▼
┌─────────────────────────────────────────────────────────────────┐
│ 2. GAZEBO-ROS BRIDGE (vrx_gz package)                           │
│    - Converts Gazebo messages to ROS 2 messages                 │
│    - File: vrx_gz/src/vrx_gz/payload_bridges.py                 │
│    - Function: navsat()                                          │
│    - Converts: gz.msgs.NavSat → sensor_msgs/NavSatFix           │
└───────────────────────┬─────────────────────────────────────────┘
                        │
                        ▼
┌─────────────────────────────────────────────────────────────────┐
│ 3. ROS 2 TOPIC                                                    │
│    - Topic Name: /wamv/sensors/gps/gps/fix                      │
│    - Message Type: sensor_msgs/NavSatFix                         │
│    - Contains: latitude, longitude, altitude                     │
│    - Update Rate: ~20 Hz (from GPS sensor config)                │
└───────────────────────┬─────────────────────────────────────────┘
                        │
                        ▼
┌─────────────────────────────────────────────────────────────────┐
│ 4. ROSBRIDGE SERVER                                              │
│    - Package: rosbridge_server                                   │
│    - Executable: rosbridge_websocket                             │
│    - Port: 9090 (WebSocket)                                      │
│    - Launch File: dashboard/rosbridge_fixed.launch.py           │
│    - Bridges ROS topics to WebSocket protocol                    │
│    - Protocol: WebSocket (ws://localhost:9090)                  │
└───────────────────────┬─────────────────────────────────────────┘
                        │
                        ▼
┌─────────────────────────────────────────────────────────────────┐
│ 5. ROSLIB.JS (JavaScript Library)                                │
│    - Library: roslib@1.3.0 (from CDN)                           │
│    - URL: https://cdn.jsdelivr.net/npm/roslib@1.3.0/...         │
│    - Creates WebSocket connection to rosbridge                  │
│    - Subscribes to ROS topics via WebSocket                      │
│    - Receives JSON messages from rosbridge                       │
└───────────────────────┬─────────────────────────────────────────┘
                        │
                        ▼
┌─────────────────────────────────────────────────────────────────┐
│ 6. DASHBOARD JAVASCRIPT                                         │
│    - File: dashboard/dashboard.html                              │
│    - Uses ROSLIB.Topic to subscribe                              │
│    - Updates Leaflet map with new position                       │
│    - Updates telemetry display                                   │
└─────────────────────────────────────────────────────────────────┘
```

## 🔧 APIs & Technologies Used

### 1. **ROS 2 (Robot Operating System)**
   - **Purpose**: Middleware for robot communication
   - **Topic**: `/wamv/sensors/gps/gps/fix`
   - **Message Type**: `sensor_msgs/NavSatFix`
   - **Fields**: `latitude`, `longitude`, `altitude`

### 2. **rosbridge_server**
   - **Package**: `ros-${ROS_DISTRO}-rosbridge-suite`
   - **API**: WebSocket server on port 9090
   - **Protocol**: WebSocket (ws://)
   - **Function**: Bridges ROS topics to WebSocket for web access

### 3. **ROSLIB.js**
   - **Library**: roslib@1.3.0
   - **Type**: JavaScript client library
   - **API**: 
     ```javascript
     new ROSLIB.Ros({ url: 'ws://localhost:9090' })
     new ROSLIB.Topic({ ros, name: '/topic', messageType: 'type' })
     ```
   - **Function**: Connects to rosbridge and subscribes to topics

### 4. **Leaflet.js**
   - **Library**: Leaflet 1.9.4 (mapping library)
   - **API**: 
     ```javascript
     L.map('map-id')
     L.marker([lat, lon]).addTo(map)
     marker.setLatLng([lat, lon])
     ```
   - **Function**: Displays interactive map and boat position

## 💻 Implementation Details

### Step 1: Gazebo-ROS Bridge Configuration

**File**: `vrx_gz/src/vrx_gz/payload_bridges.py`

```python
def navsat(world_name, model_name, link_name, sensor_name):
    gz_sensor_prefix = gz_prefix(world_name, model_name, link_name, sensor_name)
    ros_sensor_prefix = ros_prefix('', 'gps')
    return Bridge(
        gz_topic=f'{gz_sensor_prefix}/navsat',
        ros_topic=f'{ros_sensor_prefix}gps/fix',  # → /wamv/sensors/gps/gps/fix
        gz_type='gz.msgs.NavSat',
        ros_type='sensor_msgs/msg/NavSatFix',
        direction=BridgeDirection.GZ_TO_ROS)
```

**What it does**: 
- Gazebo GPS sensor publishes `gz.msgs.NavSat` messages
- Bridge converts them to ROS 2 `sensor_msgs/NavSatFix`
- Publishes to `/wamv/sensors/gps/gps/fix` topic

### Step 2: rosbridge Server Setup

**File**: `dashboard/rosbridge_fixed.launch.py`

```python
Node(
    package='rosbridge_server',
    executable='rosbridge_websocket',
    name='rosbridge_websocket',
    parameters=[{
        'port': 9090,
        'address': '',
        'ssl': False,
        'authenticate': False,
    }],
)
```

**What it does**:
- Starts WebSocket server on port 9090
- Listens for WebSocket connections
- Bridges ROS topics to WebSocket messages
- Converts ROS messages to JSON format

### Step 3: Dashboard JavaScript Connection

**File**: `dashboard/dashboard.html` (lines 623-657)

```javascript
// Connect to rosbridge via WebSocket
ros = new ROSLIB.Ros({
    url: 'ws://localhost:9090'
});

ros.on('connection', () => {
    console.log('✅ Connected to ROS!');
    subscribeToTopics();
});
```

**What it does**:
- Creates WebSocket connection to `ws://localhost:9090`
- Waits for connection event
- Then subscribes to topics

### Step 4: GPS Topic Subscription

**File**: `dashboard/dashboard.html` (lines 726-757)

```javascript
// Subscribe to GPS topic
const gpsTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/wamv/sensors/gps/gps/fix',
    messageType: 'sensor_msgs/NavSatFix'
});

gpsTopic.subscribe((message) => {
    if (message.latitude !== 0 && message.longitude !== 0) {
        boatState.position = {
            lat: message.latitude,
            lon: message.longitude,
            alt: message.altitude || 0
        };
        updateBoatPosition();  // Update map marker
        updatePositionTelemetry();  // Update sidebar
    }
});
```

**What it does**:
- Creates ROSLIB.Topic object for GPS topic
- Subscribes via WebSocket
- Receives JSON messages with `latitude`, `longitude`, `altitude`
- Updates boat position on map and telemetry display

### Step 5: Map Update

**File**: `dashboard/dashboard.html` (lines 1076-1104)

```javascript
function updateBoatPosition() {
    const { lat, lon } = boatState.position;
    
    // Update marker position on Leaflet map
    boatMarker.setLatLng([lat, lon]);
    
    // Add to path trail
    pathTrail.push([lat, lon]);
    
    // Update path polyline
    if (pathPolyline) {
        pathPolyline.setLatLngs(pathTrail);
    }
}
```

**What it does**:
- Updates boat marker position on Leaflet map
- Adds point to path trail (history)
- Updates polyline showing boat's path

## 📡 Message Format

### ROS Message (sensor_msgs/NavSatFix)
```yaml
header:
  stamp:
    sec: 1234567890
    nanosec: 123456789
  frame_id: "wamv/gps_wamv_link"
status:
  status: 0
  service: 1
latitude: -33.722400
longitude: 150.673900
altitude: 0.0
```

### WebSocket JSON (via rosbridge)
```json
{
  "header": {
    "stamp": { "sec": 1234567890, "nanosec": 123456789 },
    "frame_id": "wamv/gps_wamv_link"
  },
  "status": { "status": 0, "service": 1 },
  "latitude": -33.722400,
  "longitude": 150.673900,
  "altitude": 0.0
}
```

## 🔄 Real-Time Update Flow

1. **Gazebo** simulates boat movement → GPS sensor updates (~20 Hz)
2. **Gazebo-ROS Bridge** converts message → ROS topic publishes
3. **rosbridge** receives ROS message → Converts to JSON → Sends via WebSocket
4. **ROSLIB.js** receives WebSocket message → Parses JSON → Calls callback
5. **Dashboard** callback updates `boatState.position` → Updates map marker
6. **Leaflet** re-renders marker at new position → User sees movement

**Total Latency**: ~50-100ms (very low, feels real-time)

## 🎯 Key Files

| File | Purpose |
|------|---------|
| `vrx_gz/src/vrx_gz/payload_bridges.py` | Gazebo-ROS bridge configuration |
| `dashboard/rosbridge_fixed.launch.py` | rosbridge server launch file |
| `dashboard/dashboard.html` | Main dashboard with JavaScript |
| `vrx_urdf/wamv_gazebo/urdf/components/wamv_gps.xacro` | GPS sensor configuration |

## 🚀 How to Verify It's Working

1. **Check ROS topic is publishing**:
   ```bash
   ros2 topic echo /wamv/sensors/gps/gps/fix
   ```

2. **Check rosbridge is running**:
   ```bash
   lsof -i :9090
   ```

3. **Check WebSocket connection**:
   - Open browser console (F12)
   - Look for: `✅ Connected to ROS!`
   - Look for: `✅ Subscribed to /wamv/sensors/gps/gps/fix`

4. **Check messages received**:
   - Browser console should show: `🚤 Updating boat position: -33.722 150.674`
   - Map marker should move when boat moves in Gazebo

## 📚 References

- **ROS 2**: https://docs.ros.org/
- **rosbridge**: http://wiki.ros.org/rosbridge_suite
- **ROSLIB.js**: https://github.com/RobotWebTools/roslibjs
- **Leaflet**: https://leafletjs.com/
- **sensor_msgs/NavSatFix**: https://docs.ros2.org/latest/api/sensor_msgs/msg/NavSatFix.html

