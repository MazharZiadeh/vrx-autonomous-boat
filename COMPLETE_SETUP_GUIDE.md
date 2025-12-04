# 🚤 VRX Hackathon - Complete Setup Guide

## ✅ STEP 1: Verify Installation

```bash
# Check ROS 2
source /opt/ros/jazzy/setup.bash
ros2 --version

# Check Gazebo
gz sim --version

# Should show: Gazebo Sim, version 8.x
```

## ✅ STEP 2: Build Workspace (Already Done!)

```bash
cd /home/mazhar/vrx_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 🚀 STEP 3: Launch VRX with WAM-V

### Terminal 1: Launch Simulation
```bash
cd /home/mazhar/vrx_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Launch VRX with WAM-V boat
ros2 launch vrx_gz vrx_environment.launch.py world:=sydney_regatta sim_mode:=full
```

**What this does:**
- Launches Gazebo with Sydney Regatta world (ocean environment)
- Spawns a WAM-V boat with sensors
- Bridges Gazebo topics to ROS 2 topics
- `sim_mode:=full` enables ROS bridges (required for autonomy)

**Wait 10-15 seconds** for everything to initialize!

### Terminal 2: Verify Topics
```bash
cd /home/mazhar/vrx_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# List all topics
ros2 topic list

# You should see topics like:
# /wamv/pose
# /wamv/sensors/gps/gps/fix
# /wamv/sensors/imu/imu/data
# /wamv/thrusters/left/thrust
# /wamv/thrusters/right/thrust
```

### Terminal 3: Test Sensor Data
```bash
cd /home/mazhar/vrx_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Check GPS
ros2 topic echo /wamv/sensors/gps/gps/fix

# Check IMU
ros2 topic echo /wamv/sensors/imu/imu/data

# Check Pose
ros2 topic echo /wamv/pose
```

## 🤖 STEP 4: Launch Autonomy Node

### Terminal 4: Start Autonomy
```bash
cd /home/mazhar/vrx_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Launch autonomy node
ros2 run wamv_autonomy autonomy_node

# OR use launch file
ros2 launch wamv_autonomy autonomy.launch.py
```

**What happens:**
- Node subscribes to GPS, IMU, and pose
- Calculates waypoint navigation
- Publishes thrust commands to move boat
- Logs status every second

**Expected output:**
```
[INFO] [wamv_autonomy_node]: WAM-V Autonomy Node started
[INFO] [wamv_autonomy_node]: Mission: 5 waypoints
[INFO] [wamv_autonomy_node]: Origin set: <lat>, <lon>
[INFO] [wamv_autonomy_node]: Navigating to waypoint 1
[INFO] [wamv_autonomy_node]: Pos: (x, y) | Heading: 0.0° | WP 1/5 | Distance: 50.0m
```

## 🌐 STEP 5: Setup Web Dashboard (rosbridge)

### Install rosbridge
```bash
sudo apt install ros-jazzy-rosbridge-suite
```

### Terminal 5: Launch rosbridge
```bash
cd /home/mazhar/vrx_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

**This creates a WebSocket server on port 9090** for web dashboard communication.

## 📊 STEP 6: Create Web Dashboard

### Setup React Dashboard
```bash
cd /home/mazhar/final_stand
npx create-react-app wamv-dashboard
cd wamv-dashboard
npm install rosbridge-client leaflet react-leaflet
```

### Dashboard Structure
```
wamv-dashboard/
├── src/
│   ├── App.js          # Main dashboard component
│   ├── MapView.js      # Map visualization with Leaflet
│   ├── Telemetry.js    # Sensor data display
│   └── Mission.js      # Waypoint and mission status
```

## 🎯 DEMO SCENARIOS

### Scenario 1: Waypoint Navigation
- Boat navigates through 5 waypoints
- Real-time visualization on map
- Shows heading, speed, distance to waypoint

### Scenario 2: Dynamic Re-planning
- Add/remove waypoints during mission
- Obstacle avoidance (future enhancement)
- Re-route when conditions change

### Scenario 3: Mission Optimization
- Calculate optimal waypoint order
- Minimize travel distance/time
- Multi-objective optimization

## 🔧 TROUBLESHOOTING

### Gazebo doesn't launch
```bash
# Check if Gazebo is installed
gz sim --version

# If not, install:
sudo apt install gz-sim8
```

### Topics not appearing
- Make sure `sim_mode:=full` in launch command
- Wait 10-15 seconds after launch
- Check: `ros2 topic list`

### Autonomy node not moving boat
- Check if pose topic is publishing: `ros2 topic echo /wamv/pose`
- Verify thrust commands: `ros2 topic echo /wamv/thrusters/left/thrust`
- Check node logs for errors

### rosbridge connection fails
- Verify rosbridge is running: `ros2 node list | grep rosbridge`
- Check WebSocket port: `netstat -tuln | grep 9090`
- Browser console will show connection errors

## 📝 QUICK REFERENCE

### Essential Topics
- **Pose**: `/wamv/pose` (geometry_msgs/Pose)
- **GPS**: `/wamv/sensors/gps/gps/fix` (sensor_msgs/NavSatFix)
- **IMU**: `/wamv/sensors/imu/imu/data` (sensor_msgs/Imu)
- **Left Thrust**: `/wamv/thrusters/left/thrust` (std_msgs/Float64)
- **Right Thrust**: `/wamv/thrusters/right/thrust` (std_msgs/Float64)

### Manual Control Test
```bash
# Move forward
ros2 topic pub --once /wamv/thrusters/left/thrust std_msgs/msg/Float64 '{data: 500.0}'
ros2 topic pub --once /wamv/thrusters/right/thrust std_msgs/msg/Float64 '{data: 500.0}'

# Turn left
ros2 topic pub --once /wamv/thrusters/left/thrust std_msgs/msg/Float64 '{data: -500.0}'
ros2 topic pub --once /wamv/thrusters/right/thrust std_msgs/msg/Float64 '{data: 500.0}'
```

## 🎉 NEXT STEPS

1. ✅ VRX launched and verified
2. ✅ Autonomy node created and tested
3. ⏭️ Build React dashboard
4. ⏭️ Connect dashboard to rosbridge
5. ⏭️ Implement demo scenarios
6. ⏭️ Polish UI for demo

---

**Time Estimate:**
- Setup: 30 min ✅
- Autonomy: 1 hour ✅
- Dashboard: 2-3 hours
- Demo scenarios: 2-3 hours
- Polish: 1-2 hours

**Total: ~6-8 hours of focused work**

