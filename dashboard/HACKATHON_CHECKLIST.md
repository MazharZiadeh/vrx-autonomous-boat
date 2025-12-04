# 🏆 HACKATHON DEMO CHECKLIST

## Pre-Demo (Now)

- [ ] VRX launches without errors
- [ ] Boat spawns and is visible in Gazebo
- [ ] rosbridge connects (check port 9090)
- [ ] Dashboard opens in browser
- [ ] Dashboard shows "Connected" status
- [ ] Boat marker appears on map
- [ ] Boat moves when simulation runs
- [ ] Heading rotates correctly
- [ ] Telemetry updates in real-time
- [ ] MiniMap syncs with main map
- [ ] Debug panel works (press 'D')

## Demo Scenarios (Priority Order)

### Scenario 1: Basic Navigation (MUST WORK)
- [ ] Spawn boat at origin
- [ ] Boat moves to waypoint 1
- [ ] Dashboard shows real-time position
- [ ] Heading indicator rotates
- [ ] Telemetry updates

### Scenario 2: Obstacle Avoidance (NICE TO HAVE)
- [ ] Add obstacle in path
- [ ] Boat re-routes around it
- [ ] Dashboard shows replanning

### Scenario 3: Multi-Waypoint Mission (BONUS)
- [ ] Load 5 waypoints
- [ ] Boat visits them in sequence
- [ ] Progress bar updates
- [ ] ETA calculates correctly

## Backup Plans

### If VRX Breaks:
- [ ] Use fake data mode in dashboard
- [ ] Pre-recorded video of working system
- [ ] Static screenshots with voiceover

### If Dashboard Breaks:
- [ ] Use RViz to show boat
- [ ] Terminal output of telemetry
- [ ] Explain what WOULD work

### If rosbridge Breaks:
- [ ] Direct ROS 2 commands demo
- [ ] Architecture diagram explanation
- [ ] Code walkthrough

## Pitch Components
- [ ] Problem statement (30 sec)
- [ ] Solution overview (1 min)
- [ ] Live demo (3 min)
- [ ] Technical deep dive (1 min)
- [ ] Business case (30 sec)
- [ ] Q&A prep (common questions)

## Technical Talking Points
- "Industry-standard VRX simulation environment"
- "Real-time telemetry visualization"
- "Scalable architecture for production boats"
- "Web-based interface - no installation required"
- "Validated with actual autonomous boat"

## Common Judge Questions

**Q: "How does this work on a real boat?"**
A: The same ROS 2 architecture runs on our physical WAM-V. We use the same autonomy node, just swap simulation sensors for real GPS/IMU. The dashboard connects via WebSocket to the boat's onboard computer.

**Q: "What happens if GPS fails?"**
A: We have fallback to IMU-based dead reckoning and can use visual odometry from cameras. The system gracefully degrades and alerts operators.

**Q: "How do you handle network latency?"**
A: The autonomy node runs locally on the boat for real-time control. The dashboard is for monitoring - latency doesn't affect boat control.

**Q: "What's your business model?"**
A: We provide autonomous navigation systems for commercial maritime operations - shipping, research vessels, and port operations. Subscription-based software with hardware integration services.

## Quick Commands

```bash
# Launch everything
./launch_everything.sh

# Run diagnostics
./diagnose.sh

# Run integration tests
./test_integration.sh

# Stop everything
./stop_everything.sh
```

