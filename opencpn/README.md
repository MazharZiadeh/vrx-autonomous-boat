# 🗺️ OpenCPN Integration for VRX

Professional nautical charting integration with VRX simulation.

## 📦 What This Does

- **NMEA Bridge**: Converts ROS 2 topics → NMEA 0183 sentences → OpenCPN
- **Route Bridge**: Converts OpenCPN GPX routes → ROS 2 waypoints
- **Real-time Visualization**: See VRX boat position on professional nautical charts

## 🚀 Quick Start

### 1. Install OpenCPN

```bash
# Ubuntu/Debian
sudo apt update
sudo apt install opencpn

# Or download from:
# https://opencpn.org/OpenCPN/info/downloads.html
```

### 2. Configure OpenCPN

1. Launch OpenCPN: `opencpn`
2. Options (F5) → Connections → Add Connection
3. Settings:
   - **DataPort**: Network (UDP)
   - **Address**: 127.0.0.1
   - **DataPort**: 10110
   - **Protocol**: NMEA 0183
   - **Direction**: Input
   - **Show NMEA Debug Window**: ✓ (for testing)

### 3. Start NMEA Bridge

```bash
cd ~/final_stand/vrx/opencpn
source /opt/ros/jazzy/setup.bash
source ~/vrx_ws/install/setup.bash  # If VRX workspace exists
python3 nmea_bridge.py
```

### 4. Verify Connection

- OpenCPN should show boat icon at VRX coordinates
- NMEA Debug Window should show incoming sentences
- Boat position updates in real-time

## 🗺️ Route Planning

### Create Route in OpenCPN

1. Right-click on map → **Route & Mark Manager**
2. Click **New Route**
3. Click on map to add waypoints
4. Right-click to finish route
5. Right-click route → **Export** → Save as GPX

### Load Route to ROS

```bash
cd ~/final_stand/vrx/opencpn
source /opt/ros/jazzy/setup.bash
source ~/vrx_ws/install/setup.bash
python3 route_bridge.py ~/path/to/route.gpx
```

This publishes waypoints to `/wamv/mission/waypoints` topic.

### Start Autonomous Navigation

```bash
ros2 launch wamv_autonomy autonomy.launch.py
```

## 📊 Complete Demo Workflow

```bash
# Terminal 1: VRX (if not running)
cd ~/vrx_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch vrx_gz competition.launch.py world:=sydney_regatta

# Terminal 2: NMEA Bridge
cd ~/final_stand/vrx/opencpn
source /opt/ros/jazzy/setup.bash
source ~/vrx_ws/install/setup.bash
python3 nmea_bridge.py

# Terminal 3: OpenCPN
opencpn

# Terminal 4: After drawing route in OpenCPN
cd ~/final_stand/vrx/opencpn
source /opt/ros/jazzy/setup.bash
source ~/vrx_ws/install/setup.bash
python3 route_bridge.py ~/opencpn_mission.gpx

# Terminal 5: Start autonomy
ros2 launch wamv_autonomy autonomy.launch.py
```

## 🎯 One-Command Launcher

```bash
cd ~/final_stand/vrx/opencpn
./start_opencpn_demo.sh
```

## 📡 NMEA Sentences Sent

- **$GPGGA**: GPS Fix Data (position, altitude)
- **$GPRMC**: Recommended Minimum (position, speed, heading, date/time)
- **$GPVTG**: Course and Speed (heading, speed in knots/km/h)
- **$HCHDG**: Heading (compass heading)
- **$WIMWV**: Wind Speed and Angle (wind direction, speed)

## 🔧 Configuration

Edit `nmea_bridge.py` parameters:

```python
# Change OpenCPN connection
self.declare_parameter('opencpn_host', '127.0.0.1')
self.declare_parameter('opencpn_port', 10110)
self.declare_parameter('update_rate', 1.0)  # Hz
```

Or launch with parameters:

```bash
ros2 run opencpn nmea_bridge --ros-args \
    -p opencpn_host:=127.0.0.1 \
    -p opencpn_port:=10110 \
    -p update_rate:=2.0
```

## 🗺️ Download Charts

### Free Chart Sources

1. **NOAA Charts** (ENC format):
   ```bash
   mkdir -p ~/opencpn_charts
   cd ~/opencpn_charts
   wget https://www.charts.noaa.gov/ENCs/AUSENCProd.zip
   unzip AUSENCProd.zip
   ```

2. **OpenCPN Chart Downloader**:
   - Options → Charts → Chart Files → Add Directory
   - Options → Charts → Install/Update → Online Charts
   - Select region (e.g., Australia/Sydney)

3. **OpenStreetMap Charts**:
   - Built into OpenCPN
   - Options → Charts → Online Charts

## 🎨 Advanced: Instrument Panel

1. **Enable Dashboard Plugin**:
   - Options → Plugins → Dashboard → Enable

2. **Add Instruments**:
   - Right-click dashboard → Preferences
   - Add: Speed (SOG), Heading (HDG), Wind, GPS Position, Course to Waypoint

3. **Arrange Layout**:
   - Drag instruments to position
   - Resize as needed

## 🐛 Troubleshooting

### Boat Not Appearing in OpenCPN

1. Check NMEA Bridge is running: `ps aux | grep nmea_bridge`
2. Check OpenCPN connection settings (port 10110)
3. Enable NMEA Debug Window in OpenCPN
4. Check ROS topics: `ros2 topic echo /wamv/sensors/gps/gps/fix`

### No NMEA Sentences in Debug Window

1. Verify VRX is running and boat is spawned
2. Check GPS topic: `ros2 topic echo /wamv/sensors/gps/gps/fix`
3. Check NMEA Bridge logs for errors
4. Verify UDP port 10110 is not blocked by firewall

### Route Not Loading

1. Verify GPX file format: `cat route.gpx | head -20`
2. Check GPX namespace (should be `http://www.topografix.com/GPX/1/1`)
3. Verify waypoints published: `ros2 topic echo /wamv/mission/waypoints`

## 🏆 Why This Wins Hackathons

1. **Professional Tool**: OpenCPN used on real ocean-crossing boats
2. **Visual Planning**: Draw routes with mouse clicks
3. **Maritime Standards**: NMEA 0183 is industry standard
4. **Real-World Ready**: Same code works on actual boat
5. **Impressive Demo**: Multi-system integration (VRX + OpenCPN + ROS 2)

## 📝 Files

- `nmea_bridge.py`: ROS 2 → NMEA → OpenCPN bridge
- `route_bridge.py`: OpenCPN GPX → ROS 2 waypoints
- `start_opencpn_demo.sh`: One-command launcher

## 🔗 Resources

- OpenCPN: https://opencpn.org/
- NMEA 0183 Standard: https://www.nmea.org/
- VRX Documentation: https://github.com/osrf/vrx

---

**Ready to navigate like a pro!** 🗺️⛵

