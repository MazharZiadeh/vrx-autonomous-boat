#!/usr/bin/env python3
"""
AIS ROS Bridge - Connects to AIS proxy and publishes vessels to ROS
for Gazebo visualization
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import String, Header
import json
import asyncio
import websockets
from websockets.asyncio.client import connect
import threading
import math

class AISROSBridge(Node):
    def __init__(self):
        super().__init__('ais_ros_bridge')
        
        # Publisher for AIS vessel positions
        self.vessel_pub = self.create_publisher(
            PoseStamped,
            '/ais/vessels',
            10
        )
        
        # Publisher for vessel info (JSON string)
        self.vessel_info_pub = self.create_publisher(
            String,
            '/ais/vessel_info',
            10
        )
        
        # Store vessels
        self.vessels = {}  # MMSI -> vessel data
        
        # Start async WebSocket connection
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(target=self.run_async_loop, daemon=True)
        self.thread.start()
        
        # Timer to publish vessel positions
        self.create_timer(1.0, self.publish_vessels)  # 1 Hz
        
        self.get_logger().info('✅ AIS ROS Bridge started')
        self.get_logger().info('   Connecting to AIS proxy at ws://localhost:9091')
        
        # Start WebSocket connection
        asyncio.run_coroutine_threadsafe(self.connect_ais_proxy(), self.loop)
    
    def run_async_loop(self):
        """Run async event loop in separate thread"""
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()
    
    async def connect_ais_proxy(self):
        """Connect to AIS proxy WebSocket"""
        uri = "ws://localhost:9091"
        
        while True:
            try:
                async with connect(uri) as websocket:
                    self.get_logger().info('✅ Connected to AIS proxy')
                    
                    async for message in websocket:
                        try:
                            data = json.loads(message)
                            if data.get("type") == "vessel_update":
                                self.update_vessel(data["mmsi"], data["data"])
                        except json.JSONDecodeError:
                            pass
                        except Exception as e:
                            self.get_logger().warn(f'Error processing message: {e}')
                            
            except Exception as e:
                self.get_logger().warn(f'Connection error: {e}, retrying in 5s...')
                await asyncio.sleep(5)
    
    def update_vessel(self, mmsi, data):
        """Update vessel data"""
        self.vessels[mmsi] = data
    
    def publish_vessels(self):
        """Publish vessel positions to ROS"""
        if not self.vessels:
            return
        
        # Publish vessel info as JSON
        vessel_info = {
            "vessels": list(self.vessels.values())
        }
        info_msg = String()
        info_msg.data = json.dumps(vessel_info)
        self.vessel_info_pub.publish(info_msg)
        
        # Publish each vessel position
        for mmsi, vessel in self.vessels.items():
            pose_msg = PoseStamped()
            pose_msg.header = Header()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "map"
            
            # Convert lat/lon to Gazebo coordinates (Sydney Regatta)
            # Sydney Regatta center: -33.724223, 150.679736
            # Gazebo origin: approximately -532, 162 (from spawn configs)
            lat_center = -33.724223
            lon_center = 150.679736
            
            # Convert to local coordinates (approximate)
            # 1 degree lat ≈ 111,000 meters
            # 1 degree lon ≈ 111,000 * cos(lat) meters
            lat_diff = vessel["latitude"] - lat_center
            lon_diff = vessel["longitude"] - lon_center
            
            # Convert to meters
            y = lat_diff * 111000.0  # North-South
            x = lon_diff * 111000.0 * math.cos(math.radians(lat_center))  # East-West
            
            # Set position (z = 0 for water surface)
            pose_msg.pose.position.x = float(x)
            pose_msg.pose.position.y = float(y)
            pose_msg.pose.position.z = 0.0
            
            # Set orientation from heading
            heading_rad = math.radians(vessel.get("heading", 0))
            # Convert heading to quaternion (yaw only)
            pose_msg.pose.orientation.z = math.sin(heading_rad / 2.0)
            pose_msg.pose.orientation.w = math.cos(heading_rad / 2.0)
            
            # Add MMSI to frame_id for identification
            pose_msg.header.frame_id = f"ais_vessel_{mmsi}"
            
            self.vessel_pub.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AISROSBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

