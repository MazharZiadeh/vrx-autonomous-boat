# ⚡ OpenCPN Quick Start (5 Minutes)

## Step 1: Install OpenCPN (if not installed)

```bash
sudo apt install opencpn
```

## Step 2: Start Everything

```bash
cd ~/final_stand/vrx/opencpn
./start_opencpn_demo.sh
```

## Step 3: Configure OpenCPN (First Time Only)

1. OpenCPN opens → Click **Options** (wrench icon) or press `F5`
2. Go to **Connections** tab
3. Click **Add Connection**
4. Fill in:
   - **DataPort**: Network (UDP)
   - **Address**: `127.0.0.1`
   - **DataPort**: `10110`
   - **Protocol**: NMEA 0183
   - **Direction**: Input
   - **Show NMEA Debug Window**: ✓
5. Click **OK** → **Apply** → **OK**

## Step 4: Verify Connection

- You should see boat icon appear on map
- NMEA Debug Window shows incoming sentences
- Position updates in real-time

## Step 5: Create Route

1. Right-click on map → **Route & Mark Manager**
2. Click **New Route**
3. Click on map to add waypoints (at least 2)
4. Right-click → **Finish Route**
5. Right-click route → **Export** → Save as `~/opencpn_mission.gpx`

## Step 6: Load Route to Boat

```bash
cd ~/final_stand/vrx/opencpn
source /opt/ros/jazzy/setup.bash
source ~/vrx_ws/install/setup.bash
python3 route_bridge.py ~/opencpn_mission.gpx
```

## Step 7: Start Autonomy

```bash
ros2 launch wamv_autonomy autonomy.launch.py
```

## ✅ Done!

- OpenCPN shows boat following route
- VRX simulation shows physical movement
- Dashboard shows telemetry

---

**That's it! You're navigating like a pro!** 🗺️⛵

