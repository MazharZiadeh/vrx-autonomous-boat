#!/usr/bin/env python3
"""
Random Boats Spawner - Spawns random boats in Gazebo that move around
Some move slowly, some are static, all avoid terrain
"""

import sys
import os

# Import standard library modules first (before any path manipulation)
# This ensures we get the real logging module, not a shadowed one
import logging
import math
import random
import time

# Ensure current directory is not first in path (to avoid conflicts)
# This prevents local files from shadowing standard library modules
if sys.path and (sys.path[0] == '' or sys.path[0] == os.getcwd()):
    current_dir = sys.path.pop(0)

# Add ROS Python paths
ros_python_path = "/opt/ros/jazzy/lib/python3.12/site-packages"
if ros_python_path not in sys.path:
    sys.path.insert(0, ros_python_path)

# Now import rclpy (should work with ROS sourced)
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

class RandomBoat:
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

class RandomBoatsSpawner(Node):
    def __init__(self):
        super().__init__('random_boats_spawner')
        
        # Service clients - use simple_demo (the actual world name in Gazebo)
        self.spawn_client = self.create_client(SpawnEntity, '/world/simple_demo/create')
        self.set_pose_client = self.create_client(SetEntityPose, '/world/simple_demo/set_pose')
        
        # Wait for services (with timeout to avoid infinite wait)
        max_wait = 30  # 30 seconds max
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
        # Water area around Sydney Regatta: wider area to ensure boats stay in sea
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
        
        self.get_logger().info('✅ Random Boats Spawner started')
        self.get_logger().info(f'   Spawned {len(self.boats)} boats')
    
    def spawn_boats(self, num_boats=20):
        """Spawn random boats in the water area"""
        behaviors = ['static', 'circle', 'straight', 'zigzag']
        
        for i in range(num_boats):
            # Random position in water bounds
            x = random.uniform(self.water_bounds['x_min'], self.water_bounds['x_max'])
            y = random.uniform(self.water_bounds['y_min'], self.water_bounds['y_max'])
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
            
            boat = RandomBoat(f"random_boat_{i}", x, y, heading, speed, behavior)
            self.boats.append(boat)
            self.spawn_boat_model(boat)
    
    def spawn_boat_model(self, boat):
        """Spawn a boat model in Gazebo"""
        # Simple boat SDF
        boat_sdf = f'''<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{boat.name}">
    <static>false</static>
    <link name="link">
      <pose>0 0 0.3 0 0 0</pose>
      <collision name="collision">
        <geometry>
          <box>
            <size>8 2.5 0.8</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>8 2.5 0.8</size>
          </box>
        </geometry>
        <material>
          <ambient>{random.uniform(0.2, 0.8)} {random.uniform(0.2, 0.8)} {random.uniform(0.2, 0.8)} 1</ambient>
          <diffuse>{random.uniform(0.3, 0.9)} {random.uniform(0.3, 0.9)} {random.uniform(0.3, 0.9)} 1</diffuse>
        </material>
      </visual>
      <inertial>
        <mass>500</mass>
        <inertia>
          <ixx>100</ixx>
          <iyy>100</iyy>
          <izz>100</izz>
        </inertia>
      </inertial>
    </link>
  </model>
</sdf>'''
        
        # Convert heading to quaternion
        qz = math.sin(boat.heading / 2.0)
        qw = math.cos(boat.heading / 2.0)
        
        request = SpawnEntity.Request()
        request.name = boat.name
        request.sdf = boat_sdf
        request.pose.position.x = float(boat.x)
        request.pose.position.y = float(boat.y)
        request.pose.position.z = 0.0
        request.pose.orientation.z = float(qz)
        request.pose.orientation.w = float(qw)
        
        future = self.spawn_client.call_async(request)
        future.add_done_callback(lambda f, b=boat: self.spawn_callback(f, b))
    
    def spawn_callback(self, future, boat):
        """Callback when spawn completes"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'✅ Spawned {boat.name} ({boat.behavior}, {boat.speed:.1f} m/s)')
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
        node = RandomBoatsSpawner()
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

