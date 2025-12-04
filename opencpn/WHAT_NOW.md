# 🎯 What Now? - Quick Fix Guide

## 📊 Current Status:

✅ GPS topic exists  
❌ GPS NOT publishing (VRX still initializing)  
❌ NMEA Bridge NOT running  
✅ OpenCPN connection works (test sentence received!)  

## ✅ Solution: Run This Script

```bash
cd ~/final_stand/vrx/opencpn
./FIX_NOW.sh
```

This will:
1. Kill any old NMEA bridge processes
2. Wait for GPS to start publishing (up to 20 seconds)
3. Start NMEA bridge
4. Verify everything is working

## 🔍 Or Do It Manually:

### Step 1: Wait for GPS (10-20 seconds)

VRX just started, GPS needs time to initialize. Wait a bit, then check:

```bash
ros2 topic echo /wamv/sensors/gps/gps/fix --once
```

You should see:
```
latitude: -33.7224...
longitude: 150.6739...
```

### Step 2: Start NMEA Bridge

```bash
cd ~/final_stand/vrx/opencpn
source /opt/ros/jazzy/setup.bash
source ~/vrx_ws/install/setup.bash
python3 nmea_bridge.py
```

Keep this terminal open! You should see:
```
🗺️  NMEA Bridge started
📡 Sending NMEA to 127.0.0.1:10110
```

### Step 3: Check OpenCPN

1. **NMEA Debug Window** should show incoming sentences:
   ```
   $GPGGA,123456.00,3354.1234,S,15040.5678,E,1,08,1.0,0.5,M,0.0,M,,*XX
   $GPRMC,123456.00,A,3354.1234,S,15040.5678,E,2.40,187.0,041224,,,A*XX
   ```

2. **Boat icon** should appear on map

3. **Position** should update in real-time

## 🐛 If Still Not Working:

### GPS Not Publishing?

```bash
# Check if VRX is fully loaded
ps aux | grep gazebo

# Check if boat is visible in Gazebo window
# If not, spawn boat:
ros2 launch vrx_gz spawn.launch.py
```

### NMEA Bridge Not Sending?

Check the NMEA bridge terminal for errors. Common issues:
- GPS topic not found (wait longer)
- ROS not sourced (source setup.bash)
- Port 10110 blocked (check firewall)

### OpenCPN Still Shows Nothing?

1. **Verify connection settings:**
   - UDP (not TCP!)
   - 127.0.0.1 (not localhost!)
   - Port 10110
   - Input (not Output!)
   - NMEA Debug Window enabled

2. **Restart OpenCPN** after changing connection settings

3. **Test manually:**
   ```bash
   echo '$GPGGA,123456.00,3354.1234,S,15040.5678,E,1,08,1.0,0.5,M,0.0,M,,*XX' | nc -u 127.0.0.1 10110
   ```
   If you see this in OpenCPN Debug Window, connection works!

## ✅ Expected Result:

Once GPS starts publishing and NMEA bridge is running:
- ✅ OpenCPN shows boat position
- ✅ Position updates in real-time
- ✅ NMEA Debug Window shows sentences every ~1 second

---

**TL;DR: Run `./FIX_NOW.sh` and wait for GPS to initialize!** 🚤

