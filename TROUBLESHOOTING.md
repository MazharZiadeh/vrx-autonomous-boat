# 🔧 Troubleshooting Guide

## Problem: `/wamv/pose` topic not found

**Symptom:**
```bash
ros2 topic echo /wamv/pose
WARNING: topic [/wamv/pose] does not appear to be published yet
```

**Cause:** VRX simulation is not running.

**Solution:**
1. **Launch VRX FIRST** in Terminal 1:
   ```bash
   cd /home/mazhar/vrx_ws
   source /opt/ros/jazzy/setup.bash
   source install/setup.bash
   ros2 launch vrx_gz vrx_environment.launch.py world:=sydney_regatta sim_mode:=full
   ```

2. **Wait 10-15 seconds** for Gazebo to fully initialize

3. **Verify in Terminal 2:**
   ```bash
   cd /home/mazhar/vrx_ws
   source /opt/ros/jazzy/setup.bash
   source install/setup.bash
   ros2 topic list | grep wamv
   ```

   You should see topics like:
   - `/wamv/pose`
   - `/wamv/sensors/gps/gps/fix`
   - `/wamv/thrusters/left/thrust`
   - etc.

## Problem: Autonomy node exits with shutdown error

**Symptom:**
```
rclpy._rclpy_pybind11.RCLError: failed to shutdown: rcl_shutdown already called
```

**Cause:** Normal when pressing Ctrl+C - already fixed in latest version.

**Solution:** Rebuild the package:
```bash
cd /home/mazhar/vrx_ws
colcon build --symlink-install --packages-select wamv_autonomy
source install/setup.bash
```

## Problem: Gazebo window doesn't open

**Check:**
```bash
gz sim --version
```

**If not found:**
```bash
sudo apt install gz-sim8
```

## Problem: No topics at all

**Check if ROS 2 is sourced:**
```bash
echo $ROS_DISTRO
# Should output: jazzy
```

**If empty:**
```bash
source /opt/ros/jazzy/setup.bash
```

## Correct Launch Order

1. **Terminal 1:** Launch VRX (wait for Gazebo to load)
2. **Terminal 2:** Verify topics exist
3. **Terminal 3:** Launch autonomy node

## Quick Verification

```bash
# Check if VRX is running
ps aux | grep gz

# Check available topics
ros2 topic list

# Check if boat pose is updating
ros2 topic echo /wamv/pose --once
```

