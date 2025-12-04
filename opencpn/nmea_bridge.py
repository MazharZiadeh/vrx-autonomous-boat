#!/usr/bin/env python3

"""
NMEA Bridge for OpenCPN Integration

Converts ROS 2 topics → NMEA 0183 sentences → OpenCPN

"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float64
import socket
import math
from datetime import datetime, UTC
import threading


class NMEABridge(Node):
    def __init__(self):
        super().__init__('nmea_bridge')
        
        # NMEA output configuration
        self.declare_parameter('opencpn_host', '127.0.0.1')
        self.declare_parameter('opencpn_port', 10110)
        self.declare_parameter('update_rate', 1.0)  # Hz
        
        self.host = self.get_parameter('opencpn_host').value
        self.port = self.get_parameter('opencpn_port').value
        self.update_rate = self.get_parameter('update_rate').value
        
        # UDP socket for sending NMEA to OpenCPN
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # Boat state
        self.latitude = None
        self.longitude = None
        self.altitude = 0.0
        self.heading = 0.0  # degrees
        self.speed = 0.0  # m/s
        self.wind_direction = 0.0  # degrees
        self.wind_speed = 0.0  # m/s
        
        self.last_position_time = None
        self.last_position = None
        
        # Subscribers
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/wamv/sensors/gps/gps/fix',
            self.gps_callback,
            10
        )
        
        self.imu_sub = self.create_subscription(
            Imu,
            '/wamv/sensors/imu/imu/data',
            self.imu_callback,
            10
        )
        
        # Wind data (if available)
        self.wind_dir_sub = self.create_subscription(
            Float64,
            '/vrx/debug/wind/direction',
            self.wind_dir_callback,
            10
        )
        
        self.wind_speed_sub = self.create_subscription(
            Float64,
            '/vrx/debug/wind/speed',
            self.wind_speed_callback,
            10
        )
        
        # NMEA output timer
        self.timer = self.create_timer(1.0 / self.update_rate, self.send_nmea_data)
        
        self.get_logger().info(f'🗺️  NMEA Bridge started')
        self.get_logger().info(f'📡 Sending NMEA to {self.host}:{self.port}')
        self.get_logger().info(f'💡 Configure OpenCPN: Connection → Network → Protocol: UDP, Port: {self.port}')
    
    def gps_callback(self, msg):
        """Update GPS position and calculate speed"""
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.altitude = msg.altitude
        
        # Calculate speed from position changes
        current_time = self.get_clock().now()
        if self.last_position_time is not None and self.last_position is not None:
            dt = (current_time - self.last_position_time).nanoseconds / 1e9
            if dt > 0:
                distance = self.haversine_distance(
                    self.last_position[0], self.last_position[1],
                    self.latitude, self.longitude
                )
                self.speed = distance / dt  # m/s
        
        self.last_position = (self.latitude, self.longitude)
        self.last_position_time = current_time
    
    def imu_callback(self, msg):
        """Update heading from IMU"""
        q = msg.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.heading = (math.degrees(yaw) + 360) % 360
    
    def wind_dir_callback(self, msg):
        """Update wind direction"""
        self.wind_direction = (math.degrees(msg.data) + 360) % 360
    
    def wind_speed_callback(self, msg):
        """Update wind speed"""
        self.wind_speed = msg.data  # m/s
    
    def haversine_distance(self, lat1, lon1, lat2, lon2):
        """Calculate distance between GPS points (meters)"""
        R = 6371000
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)
        
        a = math.sin(dlat/2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        return R * c
    
    def calculate_checksum(self, sentence):
        """Calculate NMEA checksum"""
        checksum = 0
        for char in sentence:
            checksum ^= ord(char)
        return f'{checksum:02X}'
    
    def format_nmea(self, sentence):
        """Add NMEA framing and checksum"""
        checksum = self.calculate_checksum(sentence)
        return f'${sentence}*{checksum}\r\n'
    
    def format_lat_nmea(self, lat):
        """Format latitude for NMEA (DDMM.MMMM)"""
        deg = int(abs(lat))
        minutes = (abs(lat) - deg) * 60
        hemisphere = 'N' if lat >= 0 else 'S'
        return f'{deg:02d}{minutes:07.4f},{hemisphere}'
    
    def format_lon_nmea(self, lon):
        """Format longitude for NMEA (DDDMM.MMMM)"""
        deg = int(abs(lon))
        minutes = (abs(lon) - deg) * 60
        hemisphere = 'E' if lon >= 0 else 'W'
        return f'{deg:03d}{minutes:07.4f},{hemisphere}'
    
    def send_nmea_data(self):
        """Send NMEA sentences to OpenCPN"""
        if self.latitude is None or self.longitude is None:
            return
        
        now = datetime.now(datetime.UTC)
        time_str = now.strftime('%H%M%S.00')
        date_str = now.strftime('%d%m%y')
        
        # Prepare NMEA sentences
        sentences = []
        
        # $GPGGA - GPS Fix Data
        lat_nmea = self.format_lat_nmea(self.latitude)
        lon_nmea = self.format_lon_nmea(self.longitude)
        gpgga = f'GPGGA,{time_str},{lat_nmea},{lon_nmea},1,08,1.0,{self.altitude:.1f},M,0.0,M,,'
        sentences.append(self.format_nmea(gpgga))
        
        # $GPRMC - Recommended Minimum
        speed_knots = self.speed * 1.94384  # m/s to knots
        gprmc = f'GPRMC,{time_str},A,{lat_nmea},{lon_nmea},{speed_knots:.2f},{self.heading:.1f},{date_str},,,A'
        sentences.append(self.format_nmea(gprmc))
        
        # $GPVTG - Course and Speed
        gpvtg = f'GPVTG,{self.heading:.1f},T,,M,{speed_knots:.2f},N,{self.speed:.2f},K,A'
        sentences.append(self.format_nmea(gpvtg))
        
        # $HCHDG - Heading
        hchdg = f'HCHDG,{self.heading:.1f},,,,'
        sentences.append(self.format_nmea(hchdg))
        
        # $WIMWV - Wind Speed and Angle
        wind_speed_knots = self.wind_speed * 1.94384
        # Relative wind angle (simplified - just use absolute direction)
        relative_wind = (self.wind_direction - self.heading + 360) % 360
        wimwv = f'WIMWV,{relative_wind:.1f},R,{wind_speed_knots:.2f},N,A'
        sentences.append(self.format_nmea(wimwv))
        
        # Send all sentences
        for sentence in sentences:
            try:
                self.sock.sendto(sentence.encode(), (self.host, self.port))
            except Exception as e:
                self.get_logger().error(f'Failed to send NMEA: {e}')
                return
        
        self.get_logger().debug(f'📡 Sent NMEA: Lat={self.latitude:.6f}, Lon={self.longitude:.6f}, HDG={self.heading:.1f}°, SPD={speed_knots:.2f}kn')


def main(args=None):
    rclpy.init(args=args)
    bridge = NMEABridge()
    
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.sock.close()
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

