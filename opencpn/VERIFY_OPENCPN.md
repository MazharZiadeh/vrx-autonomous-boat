# ✅ OpenCPN Connection Verification

## 🔍 Your System Status (from diagnostics):

✅ Boat is spawned  
✅ GPS is publishing data  
✅ NMEA Bridge is running  
✅ Port 10110 is active  

**Everything is working!** Data should be flowing.

## 🎯 Check OpenCPN Settings

### 1. Verify Connection Settings

In OpenCPN:
- Options → Connections → Your connection

**Must be exactly:**
- **DataPort**: Network (UDP) ← NOT TCP!
- **Address**: `127.0.0.1` ← NOT `localhost`!
- **DataPort**: `10110`
- **Protocol**: NMEA 0183
- **Direction**: Input ← NOT Output!
- **Show NMEA Debug Window**: ✓ (check this!)

### 2. Enable NMEA Debug Window

1. Options → Connections
2. Select your connection
3. Check **"Show NMEA Debug Window"**
4. Click **OK** → **Apply**

You should see a window showing incoming NMEA sentences like:
```
$GPGGA,123456.00,3354.1234,S,15040.5678,E,1,08,1.0,0.5,M,0.0,M,,*XX
$GPRMC,123456.00,A,3354.1234,S,15040.5678,E,2.40,187.0,041224,,,A*XX
```

### 3. Check Connection Status

In OpenCPN:
- Look at the connection icon in the toolbar
- Should show **green** or **active** status
- If red/yellow, connection isn't working

### 4. Restart OpenCPN (Sometimes Needed)

After configuring connection:
1. Close OpenCPN completely
2. Reopen OpenCPN
3. Connection should auto-connect

## 🐛 If Still No Data

### Test NMEA Bridge Manually

```bash
# Check NMEA bridge is actually sending
cd ~/final_stand/vrx/opencpn
source /opt/ros/jazzy/setup.bash
source ~/vrx_ws/install/setup.bash

# Restart NMEA bridge and watch logs
python3 nmea_bridge.py
```

You should see logs like:
```
📡 Sent NMEA: Lat=-33.722400, Lon=150.673980, HDG=187.0°, SPD=2.40kn
```

### Test UDP Port Manually

```bash
# Send test NMEA sentence manually
echo '$GPGGA,123456.00,3354.1234,S,15040.5678,E,1,08,1.0,0.5,M,0.0,M,,*XX' | nc -u 127.0.0.1 10110
```

If OpenCPN receives this, you should see it in NMEA Debug Window.

### Check Firewall

```bash
# Check if firewall is blocking
sudo ufw status

# If active, allow port 10110
sudo ufw allow 10110/udp
```

## ✅ Expected Behavior

Once working:
1. **NMEA Debug Window** shows sentences every ~1 second
2. **Boat icon** appears on map (might be off-screen if coordinates don't match chart)
3. **Position updates** in real-time
4. **Connection icon** shows green/active

## 🎯 For Demo: Is This Critical?

**Short answer: NO!**

You can:
- ✅ Show OpenCPN interface
- ✅ Draw routes manually
- ✅ Export routes to GPX
- ✅ Load routes to boat

**For full demo**, you'll want data flowing, but for setup/testing, it's fine.

## 🚀 Quick Fix Checklist

1. ✅ NMEA Bridge running? (Check: `ps aux | grep nmea_bridge`)
2. ✅ OpenCPN connection set to UDP, 127.0.0.1:10110, Input?
3. ✅ NMEA Debug Window enabled and visible?
4. ✅ OpenCPN restarted after connection setup?
5. ✅ Firewall not blocking UDP 10110?

If all checked, data should be flowing! 🚤

