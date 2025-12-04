#!/usr/bin/env python3
"""
AIS Gazebo Spawner - Spawns AIS vessels as models in Gazebo
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ros_gz_interfaces.srv import SpawnEntity
import json
import math

class AISGazeboSpawner(Node):
    def __init__(self):
        super().__init__('ais_gazebo_spawner')
        
        # Service client to spawn entities
        self.spawn_client = self.create_client(SpawnEntity, '/world/simple_demo/create')
        
        # Wait for service
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /world/simple_demo/create service...')
        
        # Subscribe to AIS vessel info
        self.vessel_sub = self.create_subscription(
            String,
            '/ais/vessel_info',
            self.vessel_callback,
            10
        )
        
        # Track spawned vessels
        self.spawned_vessels = {}  # MMSI -> entity name
        
        self.get_logger().info('✅ AIS Gazebo Spawner started')
    
    def vessel_callback(self, msg):
        """Process vessel info and spawn/update in Gazebo"""
        try:
            data = json.loads(msg.data)
            vessels = data.get('vessels', [])
            
            for vessel in vessels:
                mmsi = str(vessel.get('mmsi'))
                if not mmsi:
                    continue
                
                # Convert lat/lon to Gazebo coordinates
                lat = vessel.get('latitude', 0)
                lon = vessel.get('longitude', 0)
                heading = vessel.get('heading', 0)
                
                # Sydney Regatta center: -33.724223, 150.679736
                # Gazebo origin: approximately -532, 162
                lat_center = -33.724223
                lon_center = 150.679736
                
                # Convert to local coordinates
                lat_diff = lat - lat_center
                lon_diff = lon - lon_center
                
                # Convert to meters
                y = lat_diff * 111000.0
                x = lon_diff * 111000.0 * math.cos(math.radians(lat_center))
                
                # Spawn or update vessel
                if mmsi not in self.spawned_vessels:
                    self.spawn_vessel(mmsi, x, y, heading, vessel.get('name', f'Ship_{mmsi}'))
                else:
                    # Update position (would need a separate service for this)
                    # For now, we'll just spawn once
                    pass
                    
        except json.JSONDecodeError as e:
            self.get_logger().warn(f'JSON decode error: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing vessel: {e}')
    
    def spawn_vessel(self, mmsi, x, y, heading, name):
        """Spawn a vessel model in Gazebo"""
        # Simple boat SDF (you can customize this)
        vessel_sdf = f'''<?xml version="1.0"?>
<sdf version="1.9">
  <model name="ais_vessel_{mmsi}">
    <static>false</static>
    <link name="link">
      <pose>0 0 0.5 0 0 0</pose>
      <collision name="collision">
        <geometry>
          <box>
            <size>10 3 1</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>10 3 1</size>
          </box>
        </geometry>
        <material>
          <ambient>0.8 0.2 0.2 1</ambient>
          <diffuse>0.8 0.2 0.2 1</diffuse>
        </material>
      </visual>
      <inertial>
        <mass>1000</mass>
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
        heading_rad = math.radians(heading)
        qz = math.sin(heading_rad / 2.0)
        qw = math.cos(heading_rad / 2.0)
        
        request = SpawnEntity.Request()
        request.name = f"ais_vessel_{mmsi}"
        request.sdf = vessel_sdf
        request.pose.position.x = float(x)
        request.pose.position.y = float(y)
        request.pose.position.z = 0.0
        request.pose.orientation.z = float(qz)
        request.pose.orientation.w = float(qw)
        
        future = self.spawn_client.call_async(request)
        future.add_done_callback(lambda f: self.spawn_callback(f, mmsi, name))
    
    def spawn_callback(self, future, mmsi, name):
        """Callback when spawn completes"""
        try:
            response = future.result()
            if response.success:
                self.spawned_vessels[mmsi] = f"ais_vessel_{mmsi}"
                self.get_logger().info(f'✅ Spawned AIS vessel: {name} (MMSI: {mmsi})')
            else:
                self.get_logger().warn(f'Failed to spawn vessel {mmsi}: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Error in spawn callback: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = AISGazeboSpawner()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

