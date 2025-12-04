# 🚤 VRX Autonomous Boat System

Real-time autonomous boat mission planning and execution system with web-based dashboard for VRX (Virtual RobotX) simulation.

## ✨ Features

- 🗺️ **Real-time Web Dashboard** - Mission control aesthetic with dual-map view
- 📊 **Live Telemetry** - Position, heading, speed, wind, and mission status
- 🎯 **Waypoint Navigation** - Autonomous waypoint following with obstacle avoidance
- 🔄 **Dynamic Re-planning** - Adaptive mission execution
- 🌐 **ROS 2 Integration** - Full ROS 2 Jazzy support with rosbridge WebSocket
- 🎨 **Professional UI** - Dark theme with neon accents, inspired by mission control

## 🚀 Quick Start

### Prerequisites
- ROS 2 Jazzy
- Gazebo Harmonic
- VRX workspace built

### Launch Everything

```bash
cd ~/final_stand/vrx/dashboard
./launch_everything.sh
```

This will:
1. Launch VRX environment
2. Spawn WAM-V boat
3. Start rosbridge WebSocket server
4. Start dashboard HTTP server
5. Open browser automatically

Dashboard will be available at: **http://localhost:8000/dashboard.html**

## 📁 Project Structure

```
vrx/
├── dashboard/              # Web dashboard and scripts
│   ├── dashboard.html      # Main dashboard (single file)
│   ├── launch_everything.sh # Master launcher
│   ├── diagnose.sh         # System diagnostics
│   ├── rosbridge_fixed.launch.py # Fixed rosbridge config
│   └── ...
├── vrx_ws/                 # VRX workspace (separate)
│   └── src/
│       └── wamv_autonomy/  # Autonomy node package
└── ...
```

## 🔧 System Components

### Dashboard
- **Real-time map** with Leaflet.js
- **MiniMap** overview in corner
- **Telemetry panels** with live updates
- **Debug panel** (press 'D' key)
- **Mission control aesthetic**

### Autonomy Node
- GPS-based waypoint following
- Heading and distance control
- Thrust command publishing
- ROS 2 Jazzy compatible

### Integration
- **rosbridge** for WebSocket communication
- **Fixed launch file** for ROS 2 Jazzy parameter compatibility
- **Complete error handling** and retry logic

## 📊 Dashboard Features

- ✅ Real-time boat position tracking
- ✅ Heading indicator (rotating triangle)
- ✅ Path trail visualization
- ✅ Planned path display
- ✅ Waypoint markers
- ✅ Live telemetry updates
- ✅ Connection status monitoring
- ✅ Debug panel for troubleshooting

## 🐛 Troubleshooting

### Dashboard Shows "Disconnected"
```bash
cd ~/final_stand/vrx/dashboard
source /opt/ros/jazzy/setup.bash
ros2 launch $(pwd)/rosbridge_fixed.launch.py
```

### Check System Status
```bash
cd ~/final_stand/vrx/dashboard
./diagnose.sh
```

### Stop Everything
```bash
cd ~/final_stand/vrx/dashboard
./stop_everything.sh
```

## 📝 Documentation

- `dashboard/QUICK_START.md` - Quick start guide
- `dashboard/HACKATHON_CHECKLIST.md` - Demo checklist
- `dashboard/INTEGRATION_COMPLETE.md` - Integration details
- `dashboard/SUCCESS.md` - Success indicators

## 🎯 Demo Scenarios

1. **Basic Navigation** - Waypoint following with real-time tracking
2. **Obstacle Avoidance** - Dynamic re-planning around obstacles
3. **Multi-Waypoint Mission** - Sequential waypoint navigation

## 🛠️ Development

### Adding New Features
- Dashboard: Edit `dashboard/dashboard.html`
- Autonomy: Edit `vrx_ws/src/wamv_autonomy/`
- Launch: Edit `dashboard/launch_everything.sh`

### Testing
```bash
cd ~/final_stand/vrx/dashboard
./test_integration.sh
```

## 📄 License

This project extends the VRX simulation environment. See original VRX license.

## 🙏 Acknowledgments

- VRX (Virtual RobotX) - OSRF
- ROS 2 Jazzy
- Leaflet.js for mapping
- rosbridge for WebSocket integration

---

**Status**: ✅ Fully Operational - Ready for Hackathon Demo! 🚤
