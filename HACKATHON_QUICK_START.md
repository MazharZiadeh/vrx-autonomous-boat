# VRX Hackathon Quick Start Guide

## 🚀 IMMEDIATE COMMANDS (Copy-Paste Ready)

### 1. Setup Environment (Run Once Per Terminal)
```bash
cd /home/mazhar/vrx_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

### 2. Build VRX Workspace
```bash
cd /home/mazhar/vrx_ws
colcon build --symlink-install
source install/setup.bash
```

### 3. Launch VRX with WAM-V Boat
```bash
# Launch in a simple ocean world with WAM-V
ros2 launch vrx_gz vrx_environment.launch.py world:=sydney_regatta sim_mode:=full
```

**Alternative: Launch with custom spawn position**
```bash
ros2 launch vrx_gz spawn.launch.py \
  world:=sydney_regatta \
  name:=wamv \
  model:=wamv \
  x:=0 y:=0 z:=0 \
  sim_mode:=full
```

### 4. Verify Sensor Topics (In New Terminal)
```bash
source /opt/ros/jazzy/setup.bash
source /home/mazhar/vrx_ws/install/setup.bash

# List all topics
ros2 topic list

# Check GPS data
ros2 topic echo /wamv/sensors/gps/gps/fix

# Check IMU data
ros2 topic echo /wamv/sensors/imu/imu/data

# Check wind sensor
ros2 topic echo /wamv/sensors/wind_sensor/wind_sensor

# Check boat pose
ros2 topic echo /wamv/pose
```

### 5. Check Thrust Commands (Control Topics)
```bash
# Left thruster thrust
ros2 topic echo /wamv/thrusters/left/thrust

# Right thruster thrust  
ros2 topic echo /wamv/thrusters/right/thrust

# Left thruster position (rotation)
ros2 topic echo /wamv/thrusters/left/pos

# Right thruster position (rotation)
ros2 topic echo /wamv/thrusters/right/pos
```

## 📦 CREATE AUTONOMY PACKAGE

```bash
cd /home/mazhar/vrx_ws/src
ros2 pkg create --build-type ament_python wamv_autonomy --dependencies rclpy geometry_msgs sensor_msgs std_msgs nav_msgs
```

## 🔧 TROUBLESHOOTING

**If Gazebo doesn't launch:**
- Check: `gz sim --version` (should show version 8.x)
- Install: `sudo apt install gz-sim8`

**If topics don't appear:**
- Make sure `sim_mode:=full` (enables ROS bridges)
- Wait 10-15 seconds after launch for bridges to initialize

**If build fails:**
- Install dependencies: `rosdep update && rosdep install --from-paths src --ignore-src -r -y`

## 🎯 NEXT STEPS

1. ✅ VRX launched with WAM-V
2. ✅ Sensor topics verified
3. ⏭️ Create autonomy node
4. ⏭️ Build web dashboard
5. ⏭️ Implement demo scenarios

