# 🚤 Random Boats Spawner Guide

## Overview

The Random Boats Spawner creates 8 random boats in Gazebo that move around the water area:
- **Some boats are static** (don't move)
- **Some boats move slowly** (0.5-2 m/s)
- **Some boats move normally** (2-5 m/s)
- **All boats stay in water** (never collide with terrain)

## Movement Behaviors

1. **Static** - Boats that don't move at all
2. **Circle** - Boats that move in circular patterns
3. **Straight** - Boats that move in straight lines, turning around at boundaries
4. **Zigzag** - Boats that change direction periodically

## How to Start

### Option 1: Manual Start (Recommended)

After Gazebo is running, open a new terminal and run:

```bash
cd ~/final_stand/vrx/dashboard
./start_random_boats_manual.sh
```

This will:
- Source ROS environment
- Start the spawner
- Show boat spawning status

### Option 2: Automatic Start

The spawner is included in `LAUNCH_FULL_SYSTEM.sh` but requires ROS environment to be properly sourced. If it doesn't start automatically, use Option 1.

## What You'll See

In Gazebo:
- 8 colored box models representing boats
- Boats moving in different patterns
- Some boats stationary, some moving

In the logs:
```
✅ Spawned random_boat_0 (circle, 3.2 m/s)
✅ Spawned random_boat_1 (static, 0.0 m/s)
✅ Spawned random_boat_2 (straight, 1.5 m/s)
...
```

## Configuration

Edit `random_boats_spawner.py` to change:

- **Number of boats**: Change `num_boats=8` in `spawn_boats()`
- **Water bounds**: Modify `self.water_bounds` dictionary
- **Speed ranges**: Adjust speed values in `spawn_boats()`
- **Behaviors**: Modify the `behaviors` list

## Troubleshooting

### "Cannot import rclpy"
**Solution**: Make sure ROS is sourced:
```bash
source /opt/ros/jazzy/setup.bash
```

### "Waiting for /world/simple_demo/create service"
**Solution**: Make sure Gazebo/VRX is running first. The spawner needs Gazebo to be active.

### Boats not appearing
**Solution**: 
1. Check logs: `tail -f /tmp/random_boats.log`
2. Verify Gazebo is running: `ps aux | grep gazebo`
3. Check service exists: `ros2 service list | grep create`

### Boats spawning outside water
**Solution**: Adjust `water_bounds` in the script to match your world's water area.

## Files

- `random_boats_spawner.py` - Main spawner script
- `start_random_boats_manual.sh` - Manual launcher
- `/tmp/random_boats.log` - Log file

