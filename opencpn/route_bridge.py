#!/usr/bin/env python3

"""
Route Bridge - Convert OpenCPN GPX routes to ROS waypoints

"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
import xml.etree.ElementTree as ET
import sys


class RouteBridge(Node):
    def __init__(self):
        super().__init__('route_bridge')
        
        self.waypoints_pub = self.create_publisher(
            PoseArray,
            '/wamv/mission/waypoints',
            10
        )
        
        self.get_logger().info('🗺️  Route Bridge ready')
    
    def load_gpx(self, gpx_file):
        """Load waypoints from GPX file"""
        tree = ET.parse(gpx_file)
        root = tree.getroot()
        
        # GPX namespace
        ns = {'gpx': 'http://www.topografix.com/GPX/1/1'}
        
        waypoints = []
        for wpt in root.findall('.//gpx:wpt', ns):
            lat = float(wpt.get('lat'))
            lon = float(wpt.get('lon'))
            waypoints.append({'lat': lat, 'lon': lon})
        
        return waypoints
    
    def publish_waypoints(self, waypoints):
        """Publish waypoints to ROS"""
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = 'world'
        
        for wp in waypoints:
            pose = Pose()
            pose.position.x = wp['lat']
            pose.position.y = wp['lon']
            pose.position.z = 0.0
            pose_array.poses.append(pose)
        
        self.waypoints_pub.publish(pose_array)
        self.get_logger().info(f'✅ Published {len(waypoints)} waypoints from OpenCPN route')


def main(args=None):
    rclpy.init(args=args)
    bridge = RouteBridge()
    
    if len(sys.argv) < 2:
        bridge.get_logger().error('Usage: python3 route_bridge.py <gpx_file>')
        rclpy.shutdown()
        return
    
    gpx_file = sys.argv[1]
    
    try:
        waypoints = bridge.load_gpx(gpx_file)
        bridge.publish_waypoints(waypoints)
        rclpy.spin_once(bridge, timeout_sec=1.0)
    except Exception as e:
        bridge.get_logger().error(f'Failed to load GPX: {e}')
    
    bridge.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

