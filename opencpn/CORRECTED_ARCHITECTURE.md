# 🏗️ CORRECTED ARCHITECTURE: OpenCPN Integration

## 🔴 What Was Wrong

1. **Wrong Control Topics**: Using `/wamv/thrusters/left/thrust` (doesn't exist)
2. **Missing Route Listener**: No way to receive waypoints from OpenCPN
3. **Backwards Understanding**: Expected OpenCPN to control boat (it doesn't)

## ✅ Correct Architecture

```
VRX/Gazebo (Simulated Boat)
    ↓ (ROS2 Topics: GPS, IMU, etc.)
    ↓
ROS2 NMEA Bridge (nmea_bridge.py)
    ↓ (Converts ROS → NMEA 0183: $GPGGA, $GPRMC, $HCHDG)
    ↓ (UDP Port 10110)
    ↓
OpenCPN (Displays boat position, you draw routes)
    ↓ (User activates route → OpenCPN generates)
    ↓ ($ECAPB, $ECRMB, $ECRMC, $ECXTE via UDP Port 10111)
    ↓
ROS2 Route Listener (opencpn_route_listener.py)
    ↓ (Parses NMEA → ROS2 /wamv/mission/waypoints)
    ↓
Autonomy Node (waypoint_navigator.py)
    ↓ (Calculates thrust commands)
    ↓ (CORRECT Topics: /model/wamv/joint/.../cmd_thrust)
    ↓
Gazebo Thrusters
    ↓
Boat moves in simulation
```

## 🎯 Key Insights

**OpenCPN is ONLY:**
- ✅ Visual map display (shows boat position)
- ✅ Route planning tool (you draw routes with mouse)
- ✅ Navigation sentence generator (outputs waypoints when route activated)

**OpenCPN is NOT:**
- ❌ An autopilot (doesn't control motors)
- ❌ A controller (doesn't compute thrust)
- ❌ A real-time feedback system (doesn't close the loop)

## 📋 Correct Topics

### GPS Input to OpenCPN:
- **Port**: 10110 (UDP)
- **Direction**: Input (OpenCPN receives)
- **Sentences**: $GPGGA, $GPRMC, $HCHDG

### Route Output from OpenCPN:
- **Port**: 10111 (UDP)
- **Direction**: Output (OpenCPN sends)
- **Sentences**: $ECAPB, $ECRMB, $ECRMC, $ECXTE
- **Filter**: Output only these sentences

### Correct Gazebo Thrust Topics:
```python
# Direct thrust (for propulsion):
/model/wamv/joint/left_engine_propeller_joint/cmd_thrust
/model/wamv/joint/right_engine_propeller_joint/cmd_thrust

# Joint position (for steering angle - optional):
/wamv/left/thruster/joint/cmd_pos
/wamv/right/thruster/joint/cmd_pos
```

## 🚀 Complete Setup

### Step 1: Start VRX
```bash
cd ~/vrx_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch vrx_gz competition.launch.py world:=sydney_regatta
```

### Step 2: Start NMEA Bridge (GPS → OpenCPN)
```bash
cd ~/final_stand/vrx/opencpn
source /opt/ros/jazzy/setup.bash
source ~/vrx_ws/install/setup.bash
python3 nmea_bridge.py
```

### Step 3: Start Route Listener (OpenCPN → ROS)
```bash
cd ~/final_stand/vrx/opencpn
source /opt/ros/jazzy/setup.bash
source ~/vrx_ws/install/setup.bash
python3 opencpn_route_listener.py
```

### Step 4: Start Autonomy Node
```bash
cd ~/vrx_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch wamv_autonomy autonomy.launch.py
```

### Step 5: Configure OpenCPN

**Connection 1: Receive GPS (Input)**
- Protocol: UDP
- Address: 127.0.0.1
- Port: 10110
- Direction: **Input**
- Show NMEA Debug Window: ✓

**Connection 2: Send Route (Output)**
- Protocol: UDP
- Address: 127.0.0.1
- Port: 10111
- Direction: **Output**
- Filter: `ECAPB,ECRMB,ECRMC,ECXTE`

### Step 6: Use OpenCPN

1. **Verify boat position** appears on map
2. **Draw route**: Right-click → Mark → Create waypoints
3. **Create route**: Right-click first waypoint → "Create Route"
4. **Activate route**: Right-click route → "Activate Route"
5. **Watch**: Terminal 3 should show "New waypoint" messages
6. **Watch**: Gazebo - boat should start moving!

## 🔧 Fixed Files

- ✅ `opencpn_route_listener.py` - Receives waypoints from OpenCPN
- ✅ `waypoint_navigator.py` - Uses correct Gazebo topics
- ✅ `nmea_bridge.py` - Sends GPS to OpenCPN (already correct)

## 📚 References

- OpenCPN2ROS: https://github.com/schvarcz/OpenCPN2ROS
- VRX Gazebo Topics: Official VRX documentation
- NMEA 0183 Standard: Maritime navigation protocol

---

**This is the correct architecture. Follow these steps and it will work!** 🚤

