# 🚀 QUICK START GUIDE

## ✅ rosbridge Installed!

Great! Now let's get everything running.

## 📍 Scripts Location

All scripts are in the `dashboard/` directory:
```bash
cd ~/final_stand/vrx/dashboard
```

## 🎯 Launch Everything (One Command!)

```bash
cd ~/final_stand/vrx/dashboard
./launch_everything.sh
```

This will:
1. Launch VRX environment (20s wait)
2. Spawn WAM-V boat (10s wait)
3. Start rosbridge (5s wait)
4. Start dashboard server
5. Open browser automatically

**Total time: ~40 seconds**

## 🔍 Run Diagnostics

```bash
cd ~/final_stand/vrx/dashboard
./diagnose.sh
```

## 🧪 Test Integration

```bash
cd ~/final_stand/vrx/dashboard
./test_integration.sh
```

## 🛑 Stop Everything

```bash
cd ~/final_stand/vrx/dashboard
./stop_everything.sh
```

## 📊 Current Status (from diagnostic)

- ✅ ROS 2 Jazzy installed
- ✅ VRX workspace built
- ✅ rosbridge installed
- ⚠️ VRX not running (need to launch)
- ⚠️ rosbridge not running (will start with launch script)

## 🎬 Next Step

Run the master launcher:
```bash
cd ~/final_stand/vrx/dashboard
./launch_everything.sh
```

Then open: http://localhost:8000/dashboard.html
