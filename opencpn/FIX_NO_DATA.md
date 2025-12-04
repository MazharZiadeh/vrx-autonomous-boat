# 🔧 Fix: OpenCPN Shows No Data

## ✅ Good News: This is Normal!

If you just configured OpenCPN but haven't started the NMEA bridge yet, **this is expected**. OpenCPN is waiting for data.

## 🔍 Quick Diagnosis

Run this to check everything:

```bash
cd ~/final_stand/vrx/opencpn
./check_connection.sh
```

This will tell you:
- ✅ Is VRX running?
- ✅ Is boat spawned?
- ✅ Is GPS publishing?
- ✅ Is NMEA bridge running?

## 🚀 Quick Fix (3 Steps)

### Step 1: Make Sure VRX is Running

```bash
# Terminal 1: VRX (if not running)
cd ~/vrx_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch vrx_gz competition.launch.py world:=sydney_regatta
```

**Wait 20 seconds**, then:

```bash
# Terminal 2: Spawn boat
cd ~/vrx_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch vrx_gz spawn.launch.py
```

### Step 2: Start NMEA Bridge

```bash
# Terminal 3: NMEA Bridge
cd ~/final_stand/vrx/opencpn
source /opt/ros/jazzy/setup.bash
source ~/vrx_ws/install/setup.bash
python3 nmea_bridge.py
```

You should see:
```
🗺️  NMEA Bridge started
📡 Sending NMEA to 127.0.0.1:10110
```

### Step 3: Verify in OpenCPN

1. **Open NMEA Debug Window** (if not already open):
   - Options → Connections → Your connection → Show NMEA Debug Window ✓

2. **You should see sentences like:**
   ```
   $GPGGA,123456.00,3354.1234,S,15040.5678,E,1,08,1.0,0.5,M,0.0,M,,*XX
   $GPRMC,123456.00,A,3354.1234,S,15040.5678,E,2.40,187.0,041224,,,A*XX
   ```

3. **Boat icon should appear** on the map at VRX coordinates

## 🐛 Troubleshooting

### Problem: NMEA Bridge Says "No GPS Data"

**Solution:** VRX not running or boat not spawned
```bash
# Check if VRX is running
ps aux | grep gazebo

# Check if boat is spawned
ros2 topic list | grep gps
```

### Problem: NMEA Bridge Running But OpenCPN Shows Nothing

**Check:**
1. OpenCPN connection settings:
   - Address: `127.0.0.1` (not `localhost`)
   - Port: `10110`
   - Protocol: `NMEA 0183`
   - Direction: `Input` (not Output!)

2. Firewall blocking UDP:
   ```bash
   sudo ufw status
   # If active, allow port 10110:
   sudo ufw allow 10110/udp
   ```

3. NMEA Debug Window shows errors:
   - Check for "Connection refused" or "Timeout"
   - Verify NMEA bridge is actually sending (check logs)

### Problem: GPS Coordinates Wrong in OpenCPN

**This is OK for now!** VRX uses simulation coordinates that might not match real-world charts. The boat will still move correctly relative to waypoints.

## ✅ Expected Behavior

Once everything is running:

1. **NMEA Bridge logs** show:
   ```
   📡 Sent NMEA: Lat=-33.722800, Lon=150.674000, HDG=187.0°, SPD=2.40kn
   ```

2. **OpenCPN NMEA Debug Window** shows:
   - Incoming sentences every ~1 second
   - No errors

3. **OpenCPN Map** shows:
   - Boat icon at current position
   - Position updates in real-time
   - Heading indicator rotates

## 🎯 For Demo: Is This a Problem?

**Short answer: NO, not a problem for now!**

You can:
- ✅ Show OpenCPN interface (even without data)
- ✅ Draw routes manually
- ✅ Export routes to GPX
- ✅ Load routes to boat

**For full demo**, you'll want data flowing, but for setup/testing, it's fine.

## 🚀 One-Command Fix

If you want everything running at once:

```bash
cd ~/final_stand/vrx/opencpn
./start_opencpn_demo.sh
```

This starts NMEA bridge automatically. Then just:
1. Make sure VRX is running
2. Make sure boat is spawned
3. OpenCPN should start receiving data

---

**TL;DR: No data is normal if NMEA bridge isn't running. Start it and you're good!** 🚤

