#!/usr/bin/env python3
"""
Arrow Key Teleop for VRX Main Boat (WAM-V)
Controls the main boat using arrow keys on keyboard.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import sys
import termios
import tty
import select

class ArrowKeyTeleop(Node):
    def __init__(self):
        super().__init__('arrow_key_teleop')
        
        # Publishers for MAIN BOAT thrusters
        self.left_thrust_pub = self.create_publisher(
            Float64, '/wamv/thrusters/left/thrust', 10
        )
        self.right_thrust_pub = self.create_publisher(
            Float64, '/wamv/thrusters/right/thrust', 10
        )
        self.left_pos_pub = self.create_publisher(
            Float64, '/wamv/thrusters/left/pos', 10
        )
        self.right_pos_pub = self.create_publisher(
            Float64, '/wamv/thrusters/right/pos', 10
        )
        
        # Control parameters
        self.forward_thrust = 600.0
        self.turn_thrust = 400.0
        
        # Save terminal settings
        self.settings = termios.tcgetattr(sys.stdin)
        
        self.get_logger().info("""
        ╔═══════════════════════════════════╗
        ║   ARROW KEY BOAT CONTROL          ║
        ║   (MAIN BOAT ONLY)                ║
        ╠═══════════════════════════════════╣
        ║   ↑  : Forward                    ║
        ║   ↓  : Backward                   ║
        ║   ←  : Turn Left                  ║
        ║   →  : Turn Right                 ║
        ║   Space : Stop                   ║
        ║   q  : Quit                       ║
        ╚═══════════════════════════════════╝
        
        Make sure this terminal is focused to use arrow keys!
        """)
        
    def get_key(self):
        """Get keyboard input (non-blocking)"""
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def publish_thrust(self, left, right):
        """Publish thrust commands to both thrusters"""
        left_msg = Float64()
        left_msg.data = left
        self.left_thrust_pub.publish(left_msg)
        
        right_msg = Float64()
        right_msg.data = right
        self.right_thrust_pub.publish(right_msg)
        
        # Keep thrusters straight
        pos_msg = Float64()
        pos_msg.data = 0.0
        self.left_pos_pub.publish(pos_msg)
        self.right_pos_pub.publish(pos_msg)
    
    def forward(self):
        """Move forward"""
        self.get_logger().info('Forward ↑')
        self.publish_thrust(self.forward_thrust, self.forward_thrust)
    
    def backward(self):
        """Move backward"""
        self.get_logger().info('Backward ↓')
        self.publish_thrust(-self.forward_thrust, -self.forward_thrust)
    
    def turn_left(self):
        """Turn left"""
        self.get_logger().info('Turn Left ←')
        self.publish_thrust(-self.turn_thrust, self.turn_thrust)
    
    def turn_right(self):
        """Turn right"""
        self.get_logger().info('Turn Right →')
        self.publish_thrust(self.turn_thrust, -self.turn_thrust)
    
    def stop(self):
        """Stop all thrusters"""
        self.get_logger().info('Stop')
        self.publish_thrust(0.0, 0.0)
    
    def run(self):
        """Main control loop"""
        try:
            while rclpy.ok():
                key = self.get_key()
                
                # Arrow keys send escape sequences
                if key == '\x1b':  # ESC sequence
                    key2 = sys.stdin.read(1)
                    if key2 == '[':
                        key3 = sys.stdin.read(1)
                        
                        if key3 == 'A':    # Up arrow
                            self.forward()
                        elif key3 == 'B':  # Down arrow
                            self.backward()
                        elif key3 == 'D':  # Left arrow
                            self.turn_left()
                        elif key3 == 'C':  # Right arrow
                            self.turn_right()
                
                elif key == ' ':  # Spacebar
                    self.stop()
                
                elif key == 'q':  # Quit
                    self.get_logger().info('Quitting...')
                    break
                
                # Small delay to prevent CPU overload
                rclpy.spin_once(self, timeout_sec=0.1)
        
        except KeyboardInterrupt:
            pass
        
        finally:
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            self.stop()

def main():
    rclpy.init()
    node = ArrowKeyTeleop()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

