#!/usr/bin/env python3
"""
ROS 2 node that publishes GPS coordinates for all spawned WAM-V boats.
Uses service calls to query model poses from Gazebo.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from ros_gz_interfaces.srv import EntityPose
import math

# Sydney Regatta origin (same as main boat)
SYDNEY_ORIGIN_LAT = -33.724223
SYDNEY_ORIGIN_LON = 150.679736
METERS_PER_DEG_LAT = 111000.0
METERS_PER_DEG_LON = 111000.0 * math.cos(math.radians(SYDNEY_ORIGIN_LAT))

class SpawnedBoatsPublisher(Node):
    def __init__(self):
        super().__init__('spawned_boats_publisher')
        
        # Dictionary to store publishers for each boat
        self.boat_publishers = {}
        
        # Service client for getting entity poses
        self.world_name = 'simple_demo'  # Try this first
        self.pose_client = self.create_client(
            EntityPose,
            f'/world/{self.world_name}/pose'
        )
        
        # Wait for service
        if not self.pose_client.wait_for_service(timeout_sec=5.0):
            # Try alternative world name
            self.world_name = 'sydney_regatta'
            self.pose_client = self.create_client(
                EntityPose,
                f'/world/{self.world_name}/pose'
            )
            if not self.pose_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().warn('⚠️ Could not find pose service, will retry...')
        
        # Timer to periodically query boat poses (1 Hz)
        self.create_timer(1.0, self.update_boat_positions)
        
        self.get_logger().info('✅ Spawned Boats Publisher started')
        self.get_logger().info(f'   Using world: {self.world_name}')
        self.get_logger().info('   Looking for boats: wamv_boat_1 through wamv_boat_20')
    
    def update_boat_positions(self):
        """Periodically query and publish GPS for all spawned boats"""
        # Query each boat
        for i in range(1, 21):
            boat_name = f'wamv_boat_{i}'
            self.query_boat_pose(boat_name)
    
    def query_boat_pose(self, boat_name):
        """Query a single boat's pose and publish GPS"""
        try:
            # Create request
            request = EntityPose.Request()
            request.entity.name = boat_name
            
            # Call service (async)
            future = self.pose_client.call_async(request)
            
            # Process result when ready
            future.add_done_callback(lambda f, name=boat_name: self.pose_callback(f, name))
            
        except Exception as e:
            # Silently ignore errors (boat might not exist)
            pass
    
    def pose_callback(self, future, boat_name):
        """Process pose service response"""
        try:
            response = future.result()
            if response and response.pose:
                pose = response.pose
                
                # Convert Gazebo coordinates to lat/lon
                x = pose.position.x
                y = pose.position.y
                
                lat = SYDNEY_ORIGIN_LAT + (y / METERS_PER_DEG_LAT)
                lon = SYDNEY_ORIGIN_LON + (x / METERS_PER_DEG_LON)
                
                # Create publisher if it doesn't exist
                if boat_name not in self.boat_publishers:
                    topic_name = f'/{boat_name}/sensors/gps/gps/fix'
                    pub = self.create_publisher(NavSatFix, topic_name, 10)
                    self.boat_publishers[boat_name] = pub
                    self.get_logger().info(f'   Created GPS publisher for {boat_name} at {topic_name}')
                
                # Publish GPS fix
                gps_msg = NavSatFix()
                gps_msg.header.stamp = self.get_clock().now().to_msg()
                gps_msg.header.frame_id = 'gps'
                gps_msg.latitude = lat
                gps_msg.longitude = lon
                gps_msg.altitude = 0.0
                gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
                
                self.boat_publishers[boat_name].publish(gps_msg)
                
        except Exception as e:
            # Silently ignore errors (boat might not exist or service unavailable)
            pass

def main(args=None):
    rclpy.init(args=args)
    node = SpawnedBoatsPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

