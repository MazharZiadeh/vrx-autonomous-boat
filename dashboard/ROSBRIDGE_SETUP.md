# 🌐 ROSBridge Setup Guide

## Step 1: Install rosbridge_server

### For ROS 2 Jazzy (Current VRX)
```bash
sudo apt update
sudo apt install ros-jazzy-rosbridge-suite
```

### For ROS 2 Humble (If using humble branch)
```bash
sudo apt install ros-humble-rosbridge-suite
```

## Step 2: Launch VRX Simulation (Terminal 1)

```bash
cd /home/mazhar/vrx_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Launch VRX with boat
ros2 launch vrx_gz vrx_environment.launch.py world:=sydney_regatta sim_mode:=bridge
```

**Wait 10-15 seconds for world to load**, then in **Terminal 2** spawn boat:
```bash
cd /home/mazhar/vrx_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch vrx_gz spawn.launch.py world:=sydney_regatta name:=wamv model:=wamv x:=0 y:=0 z:=0 sim_mode:=full
```

## Step 3: Launch rosbridge WebSocket Server (Terminal 3)

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

## Step 4: Verify Connection

### Test in Browser Console

1. Open any webpage
2. Press **F12** to open developer console
3. Paste this code:

```javascript
var ros = new ROSLIB.Ros({ url: 'ws://localhost:9090' });
ros.on('connection', () => console.log('✅ Connected to ROS!'));
ros.on('error', (error) => console.log('❌ Error:', error));
ros.on('close', () => console.log('⚠️ Disconnected from ROS'));
```

**Expected:** Console shows "✅ Connected to ROS!"

### Test with Command Line

```bash
# Check if rosbridge is running
ps aux | grep rosbridge

# Check if port 9090 is listening
netstat -tuln | grep 9090
```

## Step 5: List Available Topics (Terminal 4)

```bash
cd /home/mazhar/vrx_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 topic list
```

### Key Topics to Look For

| Topic | Type | Description |
|-------|------|-------------|
| `/wamv/sensors/gps/gps/fix` | `sensor_msgs/NavSatFix` | GPS position |
| `/wamv/sensors/imu/imu/data` | `sensor_msgs/Imu` | IMU data (heading) |
| `/wamv/pose` | `geometry_msgs/Pose` | Boat pose |
| `/wamv/thrusters/left/thrust` | `std_msgs/Float64` | Left thruster command |
| `/wamv/thrusters/right/thrust` | `std_msgs/Float64` | Right thruster command |
| `/vrx/debug/wind/direction` | `std_msgs/Float32` | Wind direction |
| `/vrx/debug/wind/speed` | `std_msgs/Float32` | Wind speed |

### Verify Topics Are Publishing

```bash
# Check GPS
ros2 topic echo /wamv/sensors/gps/gps/fix --once

# Check Pose
ros2 topic echo /wamv/pose --once

# Check IMU
ros2 topic echo /wamv/sensors/imu/imu/data --once
```

## 🐛 Troubleshooting

### Problem: "rosbridge_server" not found
**Solution:**
```bash
sudo apt update
sudo apt install ros-jazzy-rosbridge-suite
```

### Problem: WebSocket connection fails
**Solution:**
1. Check rosbridge is running: `ps aux | grep rosbridge`
2. Check port 9090: `netstat -tuln | grep 9090`
3. Restart rosbridge

### Problem: No topics visible
**Solution:**
1. Make sure VRX is running with `sim_mode:=full` or `sim_mode:=bridge`
2. Make sure boat is spawned
3. Wait 10-15 seconds after spawning for topics to appear

### Problem: Topics not updating
**Solution:**
- Check if simulation is paused in Gazebo
- Verify boat is actually moving
- Check topic rate: `ros2 topic hz /wamv/sensors/gps/gps/fix`

## ✅ Quick Test Script

Save this as `test_rosbridge.html` and open in browser:

```html
<!DOCTYPE html>
<html>
<head>
    <title>ROS Bridge Test</title>
    <script src="https://cdn.jsdelivr.net/npm/roslib@1.3.0/build/roslib.min.js"></script>
</head>
<body>
    <h1>ROS Bridge Connection Test</h1>
    <div id="status">Connecting...</div>
    <div id="topics"></div>
    
    <script>
        const ros = new ROSLIB.Ros({ url: 'ws://localhost:9090' });
        
        ros.on('connection', () => {
            document.getElementById('status').textContent = '✅ Connected!';
            document.getElementById('status').style.color = 'green';
        });
        
        ros.on('error', (error) => {
            document.getElementById('status').textContent = '❌ Error: ' + error;
            document.getElementById('status').style.color = 'red';
        });
        
        ros.on('close', () => {
            document.getElementById('status').textContent = '⚠️ Disconnected';
            document.getElementById('status').style.color = 'orange';
        });
    </script>
</body>
</html>
```

## 📝 Complete Launch Sequence

**Terminal 1:** VRX World
```bash
cd /home/mazhar/vrx_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch vrx_gz vrx_environment.launch.py world:=sydney_regatta sim_mode:=bridge
```

**Terminal 2:** Spawn Boat (after 15 seconds)
```bash
cd /home/mazhar/vrx_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch vrx_gz spawn.launch.py world:=sydney_regatta name:=wamv model:=wamv x:=0 y:=0 z:=0 sim_mode:=full
```

**Terminal 3:** rosbridge
```bash
cd /home/mazhar/vrx_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

**Terminal 4:** Dashboard Server
```bash
cd /home/mazhar/final_stand/vrx/dashboard
python3 -m http.server 8000
```

Then open: **http://localhost:8000/dashboard.html**

