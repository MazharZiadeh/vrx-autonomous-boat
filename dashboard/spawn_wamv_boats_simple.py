#!/usr/bin/env python3
"""
Simple WAM-V Boats Spawner - Uses ros2 service calls to spawn WAM-V boats
Same model as main boat, but with different names (non-main)
"""

import sys
import os
import subprocess
import random
import math
import time

# Source ROS environment
os.environ['ROS_DOMAIN_ID'] = '0'

def spawn_wamv_boat(name, x, y, yaw):
    """Spawn a WAM-V boat using ros2 service call"""
    # Use model URI (same as main boat)
    # Escape quotes for SDF
    sdf_content = f'<sdf version="1.9"><include><name>{name}</name><uri>model://wam-v</uri><pose>{x} {y} 0 0 0 {yaw}</pose></include></sdf>'
    
    # Convert yaw to quaternion
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    
    # Call spawn service with correct format
    service_request = f'''entity_factory:
  name: "{name}"
  allow_renaming: false
  sdf: "{sdf_content}"
  sdf_filename: ""
  clone_name: ""
  pose:
    position:
      x: {x}
      y: {y}
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: {qz}
      w: {qw}
  relative_to: "world"'''
    
    cmd = [
        'ros2', 'service', 'call', '/world/simple_demo/create',
        'ros_gz_interfaces/srv/SpawnEntity',
        service_request
    ]
    
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
        if result.returncode == 0 and 'success: True' in result.stdout:
            return True
        return False
    except Exception as e:
        return False

def main():
    print("=" * 50)
    print("🚤 Spawning WAM-V Boats (Same as Main Boat)")
    print("=" * 50)
    print("")
    
    # Wait for Gazebo service
    print("Waiting for Gazebo service...")
    for i in range(30):
        result = subprocess.run(['ros2', 'service', 'list'], 
                              capture_output=True, text=True, timeout=2)
        if '/world/simple_demo/create' in result.stdout:
            print("✅ Gazebo service found!")
            break
        time.sleep(1)
    else:
        print("❌ Gazebo service not found. Is Gazebo running?")
        return
    
    # Water bounds
    x_min, x_max = -600.0, -400.0
    y_min, y_max = 100.0, 250.0
    main_boat_x, main_boat_y = -532.0, 162.0
    
    print("")
    print("Spawning 20 WAM-V boats in water area...")
    print("")
    
    spawned = 0
    for i in range(1, 21):
        # Random position (avoid main boat area)
        while True:
            x = random.uniform(x_min, x_max)
            y = random.uniform(y_min, y_max)
            dist = math.sqrt((x - main_boat_x)**2 + (y - main_boat_y)**2)
            if dist > 50:  # At least 50m from main boat
                break
        
        yaw = random.uniform(0, 2 * math.pi)
        boat_name = f"wamv_boat_{i}"
        
        print(f"Spawning boat {i}/20: {boat_name} at ({x:.1f}, {y:.1f})...", end=' ')
        
        if spawn_wamv_boat(boat_name, x, y, yaw):
            print("✅")
            spawned += 1
        else:
            print("⚠️")
        
        time.sleep(0.3)  # Small delay
    
    print("")
    print("=" * 50)
    print(f"✅ Spawning Complete! ({spawned}/20 boats spawned)")
    print("=" * 50)
    print("")
    print("All boats are WAM-V models (same as your main boat)")
    print("They are named wamv_boat_1 through wamv_boat_20 (non-main)")
    print("")

if __name__ == '__main__':
    main()

