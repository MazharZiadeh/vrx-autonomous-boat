# 🎤 30-Second Hackathon Demo Script

## The Pitch

> "Instead of building custom navigation software, we use OpenCPN—the same open-source system used on real ocean-crossing sailboats.
>
> [Show OpenCPN with nautical chart]
>
> I can plan routes with a mouse click, just like a real captain.
>
> [Click waypoints on chart, export to boat]
>
> The route downloads to our autonomous system...
>
> [Show boat start moving in VRX]
>
> ...and our boat navigates using the same maritime protocols as professional vessels.
>
> [Show real-time position updating in OpenCPN]
>
> This is production-ready code. We're just testing it safely in simulation first."

## Visual Flow

1. **0:00-0:10**: Show OpenCPN with professional nautical chart
2. **0:10-0:20**: Draw route with mouse clicks (3-4 waypoints)
3. **0:20-0:25**: Export route, load to boat
4. **0:25-0:30**: Show boat following route in VRX + OpenCPN

## Key Talking Points

- **Professional Tool**: "OpenCPN is used on boats crossing oceans"
- **Industry Standards**: "NMEA 0183 protocols are maritime standard"
- **Real-World Ready**: "Same code works on our actual boat"
- **Safety First**: "We validate in simulation before water testing"

## Q&A Prep

**Q: "How does this work on a real boat?"**
A: "Same ROS 2 code. We swap the simulator for hardware drivers. The autonomy stack is identical—GPS, IMU, thrusters. We use this to validate before water testing."

**Q: "What if GPS fails?"**
A: "Multi-sensor fusion. IMU + visual odometry + acoustic positioning. The simulator lets us test failure modes safely."

**Q: "Why OpenCPN instead of custom software?"**
A: "OpenCPN is battle-tested on thousands of boats. Why reinvent the wheel? We focus on autonomy, not charting."

**Q: "What's your business model?"**
A: "B2B SaaS for maritime robotics companies. Simulation platform + algorithm marketplace + consulting. $50K+ per customer."

---

**Practice this 3-4 times before the demo!** 🎯

