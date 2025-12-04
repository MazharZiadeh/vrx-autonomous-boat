#!/usr/bin/env python3
"""
Twist to Thrusters Bridge for VRX Main Boat
Converts cmd_vel (Twist) messages to individual thruster commands.
Use with teleop_twist_keyboard for WASD control.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class TwistToThrusters(Node):
    def __init__(self):
        super().__init__('twist_to_thrusters')
        
        # Subscribe to cmd_vel from keyboard teleop
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Publishers for MAIN BOAT thrusters
        self.left_thrust_pub = self.create_publisher(
            Float64, 
            '/wamv/thrusters/left/thrust', 
            10
        )
        self.right_thrust_pub = self.create_publisher(
            Float64, 
            '/wamv/thrusters/right/thrust', 
            10
        )
        
        self.left_pos_pub = self.create_publisher(
            Float64, 
            '/wamv/thrusters/left/pos', 
            10
        )
        self.right_pos_pub = self.create_publisher(
            Float64, 
            '/wamv/thrusters/right/pos', 
            10
        )
        
        # Control parameters
        self.max_thrust = 800.0
        self.max_turn_rate = 0.5
        
        self.get_logger().info('✅ Twist to Thrusters bridge active for MAIN BOAT!')
        self.get_logger().info('   Listening on /cmd_vel topic')
        
    def cmd_vel_callback(self, msg):
        """Convert Twist to differential thrust commands"""
        
        # Extract linear and angular velocities
        linear_x = msg.linear.x   # Forward/backward
        angular_z = msg.angular.z  # Rotation
        
        # Differential drive equations
        # Left thruster = linear - angular
        # Right thruster = linear + angular
        left_thrust = (linear_x - angular_z) * self.max_thrust
        right_thrust = (linear_x + angular_z) * self.max_thrust
        
        # Clamp values
        left_thrust = max(-1000.0, min(1000.0, left_thrust))
        right_thrust = max(-1000.0, min(1000.0, right_thrust))
        
        # Publish thrust commands
        left_msg = Float64()
        left_msg.data = left_thrust
        self.left_thrust_pub.publish(left_msg)
        
        right_msg = Float64()
        right_msg.data = right_thrust
        self.right_thrust_pub.publish(right_msg)
        
        # Keep thrusters straight (optional - enable for articulation)
        pos_msg = Float64()
        pos_msg.data = 0.0
        self.left_pos_pub.publish(pos_msg)
        self.right_pos_pub.publish(pos_msg)
        
        self.get_logger().debug(
            f'Thrust commands - Left: {left_thrust:.1f}, Right: {right_thrust:.1f}'
        )

def main():
    rclpy.init()
    node = TwistToThrusters()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

