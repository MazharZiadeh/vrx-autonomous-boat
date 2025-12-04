#!/usr/bin/env python3

"""
OpenCPN Route Listener

Receives waypoint data from OpenCPN when a route is activated.
Listens for $ECRMB, $ECAPB sentences containing waypoint coordinates.
"""

import socket
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
import math


class OpenCPNRouteListener(Node):
    def __init__(self):
        super().__init__('opencpn_route_listener')
        
        # UDP socket to receive NMEA from OpenCPN
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('127.0.0.1', 10111))  # Different port from GPS input
        self.sock.setblocking(False)
        
        # Publisher for waypoints
        self.waypoint_pub = self.create_publisher(PoseArray, '/wamv/mission/waypoints', 10)
        
        # Timer to poll UDP socket
        self.create_timer(0.1, self.check_nmea)
        
        # Storage for route waypoints
        self.waypoints = []
        
        self.get_logger().info("🗺️  Listening for OpenCPN route on UDP 10111")
        self.get_logger().info("💡 Configure OpenCPN: Connection → UDP Output → Port 10111")
    
    def check_nmea(self):
        try:
            data, addr = self.sock.recvfrom(1024)
            sentence = data.decode('ascii').strip()
            
            # Parse ECRMB or ECAPB sentences (contain waypoint lat/lon)
            if sentence.startswith('$ECRMB') or sentence.startswith('$ECAPB'):
                self.parse_waypoint(sentence)
                
        except BlockingIOError:
            pass  # No data available
        except Exception as e:
            self.get_logger().warn(f"Error receiving NMEA: {e}")
    
    def parse_waypoint(self, sentence):
        """
        Example $ECRMB sentence:
        $ECRMB,A,0.00,L,001,002,4807.038,N,01131.000,E,1.2,234.5,5.5,V,A*23
        Fields: lat (4807.038), lat_dir (N), lon (01131.000), lon_dir (E)
        """
        fields = sentence.split(',')
        
        if len(fields) < 14:
            return
        
        try:
            # Extract lat/lon from NMEA format (DDMM.mmm)
            lat_nmea = float(fields[6])
            lat_dir = fields[7]
            lon_nmea = float(fields[8])
            lon_dir = fields[9]
            
            # Convert NMEA to decimal degrees
            lat_deg = int(lat_nmea / 100)
            lat_min = lat_nmea - (lat_deg * 100)
            lat = lat_deg + (lat_min / 60.0)
            if lat_dir == 'S':
                lat = -lat
            
            lon_deg = int(lon_nmea / 100)
            lon_min = lon_nmea - (lon_deg * 100)
            lon = lon_deg + (lon_min / 60.0)
            if lon_dir == 'W':
                lon = -lon
            
            # Create waypoint (simplified - just lat/lon, no ENU conversion here)
            pose = Pose()
            pose.position.x = lat  # Store lat in x
            pose.position.y = lon  # Store lon in y
            pose.position.z = 0.0
            
            # Add to waypoint list (deduplicate by checking if already exists)
            if not any(abs(wp.position.x - lat) < 0.0001 and abs(wp.position.y - lon) < 0.0001 
                      for wp in self.waypoints):
                self.waypoints.append(pose)
                self.get_logger().info(f"📍 New waypoint: {lat:.6f}, {lon:.6f} (Total: {len(self.waypoints)})")
                
                # Publish updated waypoint list
                waypoint_array = PoseArray()
                waypoint_array.header.stamp = self.get_clock().now().to_msg()
                waypoint_array.header.frame_id = 'map'
                waypoint_array.poses = self.waypoints
                self.waypoint_pub.publish(waypoint_array)
                
        except (ValueError, IndexError) as e:
            self.get_logger().warn(f"Failed to parse waypoint: {e}")


def main():
    rclpy.init()
    node = OpenCPNRouteListener()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.sock.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

