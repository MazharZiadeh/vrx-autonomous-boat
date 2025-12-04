#!/usr/bin/env python3
"""
WAM-V Boats Spawner - Spawns multiple WAM-V boats (same as main boat) around the area
These are non-main boats that move around in the water
"""

import sys
import os

# Import standard library modules first
import logging
import math
import random
import time

# Ensure current directory is not first in path
if sys.path and (sys.path[0] == '' or sys.path[0] == os.getcwd()):
    current_dir = sys.path.pop(0)

# Add ROS Python paths
ros_python_path = "/opt/ros/jazzy/lib/python3.12/site-packages"
if ros_python_path not in sys.path:
    sys.path.insert(0, ros_python_path)

# Import rclpy
try:
    import rclpy
    from rclpy.node import Node
except ImportError as e:
    print(f"ERROR: Cannot import rclpy: {e}")
    print("Please ensure ROS 2 is sourced:")
    print("  source /opt/ros/jazzy/setup.bash")
    sys.exit(1)

from ros_gz_interfaces.srv import SpawnEntity, SetEntityPose
from geometry_msgs.msg import Pose, Point, Quaternion

class WAMVBoat:
    def __init__(self, name, x, y, heading, speed, behavior):
        self.name = name
        self.x = x
        self.y = y
        self.heading = heading  # radians
        self.speed = speed  # m/s
        self.behavior = behavior  # 'static', 'circle', 'straight', 'zigzag'
        self.time_offset = random.uniform(0, 2 * math.pi)
        self.radius = random.uniform(50, 150)  # For circular movement
        self.center_x = x
        self.center_y = y
        self.direction = 1  # For zigzag/straight
        self.distance_traveled = 0.0

class WAMVBoatsSpawner(Node):
    def __init__(self):
        super().__init__('wamv_boats_spawner')
        
        # Service clients
        self.spawn_client = self.create_client(SpawnEntity, '/world/simple_demo/create')
        self.set_pose_client = self.create_client(SetEntityPose, '/world/simple_demo/set_pose')
        
        # Wait for services
        max_wait = 30
        waited = 0
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            waited += 1
            if waited >= max_wait:
                self.get_logger().error('Timeout waiting for Gazebo service. Is Gazebo running?')
                raise RuntimeError('Gazebo service not available')
            self.get_logger().info(f'Waiting for /world/simple_demo/create service... ({waited}/{max_wait})')
        
        waited = 0
        while not self.set_pose_client.wait_for_service(timeout_sec=1.0):
            waited += 1
            if waited >= max_wait:
                self.get_logger().error('Timeout waiting for set_pose service')
                raise RuntimeError('Set pose service not available')
            self.get_logger().info(f'Waiting for /world/simple_demo/set_pose service... ({waited}/{max_wait})')
        
        self.get_logger().info('✅ Connected to Gazebo services')
        
        # Store boats
        self.boats = []
        
        # Water bounds (Sydney Regatta area - keep boats in water)
        # Avoid main boat area (around -532, 162)
        self.water_bounds = {
            'x_min': -600.0,  # West boundary
            'x_max': -400.0,  # East boundary
            'y_min': 100.0,   # South boundary
            'y_max': 250.0    # North boundary
        }
        
        # Spawn boats (maximum 20)
        self.spawn_boats(num_boats=20)
        
        # Timer to update boat positions
        self.create_timer(0.1, self.update_boats)  # 10 Hz update rate
        
        self.get_logger().info('✅ WAM-V Boats Spawner started')
        self.get_logger().info(f'   Spawned {len(self.boats)} WAM-V boats')
    
    def spawn_boats(self, num_boats=20):
        """Spawn random WAM-V boats in the water area"""
        behaviors = ['static', 'circle', 'straight', 'zigzag']
        
        for i in range(num_boats):
            # Random position in water bounds (avoid main boat area)
            while True:
                x = random.uniform(self.water_bounds['x_min'], self.water_bounds['x_max'])
                y = random.uniform(self.water_bounds['y_min'], self.water_bounds['y_max'])
                # Make sure not too close to main boat (around -532, 162)
                dist_from_main = math.sqrt((x + 532)**2 + (y - 162)**2)
                if dist_from_main > 50:  # At least 50m from main boat
                    break
            
            heading = random.uniform(0, 2 * math.pi)
            
            # Random behavior
            behavior = random.choice(behaviors)
            
            # Speed: static = 0, slow = 0.5-2 m/s, normal = 2-5 m/s
            if behavior == 'static':
                speed = 0.0
            elif random.random() < 0.4:  # 40% slow boats
                speed = random.uniform(0.5, 2.0)
            else:
                speed = random.uniform(2.0, 5.0)
            
            boat = WAMVBoat(f"wamv_boat_{i}", x, y, heading, speed, behavior)
            self.boats.append(boat)
            self.spawn_wamv_boat(boat)
    
    def spawn_wamv_boat(self, boat):
        """Spawn a WAM-V boat model in Gazebo (same as main boat)"""
        # Use the WAM-V model URI (same as main boat)
        # We'll use an include to reference the model
        wamv_sdf = f'''<?xml version="1.0"?>
<sdf version="1.9">
  <include>
    <name>{boat.name}</name>
    <uri>model://wam-v</uri>
    <pose>{boat.x} {boat.y} 0 0 0 {boat.heading}</pose>
  </include>
</sdf>'''
        
        request = SpawnEntity.Request()
        request.name = boat.name
        request.sdf = wamv_sdf
        request.pose.position.x = float(boat.x)
        request.pose.position.y = float(boat.y)
        request.pose.position.z = 0.0
        
        # Convert heading to quaternion
        qz = math.sin(boat.heading / 2.0)
        qw = math.cos(boat.heading / 2.0)
        request.pose.orientation.z = float(qz)
        request.pose.orientation.w = float(qw)
        
        future = self.spawn_client.call_async(request)
        future.add_done_callback(lambda f, b=boat: self.spawn_callback(f, b))
    
    def spawn_callback(self, future, boat):
        """Callback when spawn completes"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'✅ Spawned WAM-V boat: {boat.name} ({boat.behavior}, {boat.speed:.1f} m/s) at ({boat.x:.1f}, {boat.y:.1f})')
            else:
                self.get_logger().warn(f'Failed to spawn {boat.name}: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Error in spawn callback: {e}')
    
    def update_boats(self):
        """Update boat positions based on their behavior"""
        current_time = time.time()
        dt = 0.1  # Time step
        
        for boat in self.boats:
            if boat.behavior == 'static':
                continue  # Don't move static boats
            
            # Update position based on behavior
            if boat.behavior == 'circle':
                # Circular motion
                angle = current_time * boat.speed / boat.radius + boat.time_offset
                boat.x = boat.center_x + boat.radius * math.cos(angle)
                boat.y = boat.center_y + boat.radius * math.sin(angle)
                boat.heading = angle + math.pi / 2  # Tangent to circle
            
            elif boat.behavior == 'straight':
                # Straight line movement
                boat.x += boat.speed * math.cos(boat.heading) * dt
                boat.y += boat.speed * math.sin(boat.heading) * dt
                boat.distance_traveled += boat.speed * dt
                
                # Reverse direction if out of bounds
                if (boat.x < self.water_bounds['x_min'] or 
                    boat.x > self.water_bounds['x_max'] or
                    boat.y < self.water_bounds['y_min'] or
                    boat.y > self.water_bounds['y_max']):
                    boat.heading += math.pi  # Turn around
                    boat.heading = boat.heading % (2 * math.pi)
            
            elif boat.behavior == 'zigzag':
                # Zigzag pattern
                boat.x += boat.speed * math.cos(boat.heading) * dt
                boat.y += boat.speed * math.sin(boat.heading) * dt
                boat.distance_traveled += boat.speed * dt
                
                # Change direction periodically
                if boat.distance_traveled > 50:  # Every 50 meters
                    boat.heading += random.uniform(-math.pi/3, math.pi/3)
                    boat.heading = boat.heading % (2 * math.pi)
                    boat.distance_traveled = 0.0
                
                # Keep in bounds
                if (boat.x < self.water_bounds['x_min'] or 
                    boat.x > self.water_bounds['x_max']):
                    boat.heading = math.pi - boat.heading
                    boat.heading = boat.heading % (2 * math.pi)
                if (boat.y < self.water_bounds['y_min'] or 
                    boat.y > self.water_bounds['y_max']):
                    boat.heading = -boat.heading
                    boat.heading = boat.heading % (2 * math.pi)
            
            # Clamp to water bounds (safety check)
            boat.x = max(self.water_bounds['x_min'], min(self.water_bounds['x_max'], boat.x))
            boat.y = max(self.water_bounds['y_min'], min(self.water_bounds['y_max'], boat.y))
            
            # Update boat pose in Gazebo
            self.update_boat_pose(boat)
    
    def update_boat_pose(self, boat):
        """Update boat position in Gazebo"""
        # Convert heading to quaternion
        qz = math.sin(boat.heading / 2.0)
        qw = math.cos(boat.heading / 2.0)
        
        request = SetEntityPose.Request()
        request.entity.name = boat.name
        request.pose.position.x = float(boat.x)
        request.pose.position.y = float(boat.y)
        request.pose.position.z = 0.0
        request.pose.orientation.z = float(qz)
        request.pose.orientation.w = float(qw)
        
        # Call async (don't wait for response)
        self.set_pose_client.call_async(request)

def main(args=None):
    # Initialize ROS
    if not rclpy.ok():
        rclpy.init(args=args)
    
    try:
        node = WAMVBoatsSpawner()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"ERROR: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if rclpy.ok():
            try:
                node.destroy_node()
            except:
                pass
            rclpy.shutdown()

if __name__ == '__main__':
    main()

