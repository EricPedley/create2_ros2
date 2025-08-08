#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import sys
import select
import termios
import tty
import threading
import time

msg = """
Control Your Create2 Robot!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
    'i': (1, 0, 0, 0),
    'o': (1, 0, 0, -1),
    'j': (0, 0, 0, 1),
    'l': (0, 0, 0, -1),
    'u': (1, 0, 0, 1),
    ',': (-1, 0, 0, 0),
    '.': (-1, 0, 0, 1),
    'm': (-1, 0, 0, -1),
    'k': (0, 0, 0, 0),  # stop key
    'O': (1, -1, 0, 0),
    'I': (1, 0, 0, 0),
    'J': (0, 1, 0, 0),
    'L': (0, -1, 0, 0),
    'U': (1, 1, 0, 0),
    '<': (-1, 0, 0, 0),
    '>': (-1, -1, 0, 0),
    'M': (-1, 1, 0, 0),
    'K': (0, 0, 0, 0),  # stop key
    't': (0, 0, 1, 0),
    'b': (0, 0, -1, 0),
}

speedBindings = {
    'q': (1.1, 1.1),
    'z': (.9, .9),
    'w': (1.1, 1),
    'x': (.9, 1),
    'e': (1, 1.1),
    'c': (1, .9),
}

def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed, turn)

class TeleopTwistStampedKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_twist_stamped_keyboard')
        
        # Publisher for TwistStamped messages
        self.publisher = self.create_publisher(
            TwistStamped, 
            '/create2_base_controller/cmd_vel', 
            10
        )
        
        # Parameters
        self.speed = 0.5
        self.turn = 1.0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.status = 0
        
        # Control variables
        self.running = True
        self.settings = termios.tcgetattr(sys.stdin)
        
        # Start publishing in a separate thread
        self.publish_thread = threading.Thread(target=self.publish_loop, daemon=True)
        self.publish_thread.start()
        
        self.get_logger().info("Create2 Teleop TwistStamped node started")
        print(msg)
        print(vels(self.speed, self.turn))
        print("Ready! Use keys to control robot, Ctrl+C to quit")

    def publish_loop(self):
        """Continuously publish twist messages"""
        rate = self.create_rate(10)  # 10 Hz
        while self.running and rclpy.ok():
            twist_stamped = TwistStamped()
            twist_stamped.header.stamp = self.get_clock().now().to_msg()
            twist_stamped.header.frame_id = "base_link"
            
            twist_stamped.twist.linear.x = self.x * self.speed
            twist_stamped.twist.linear.y = self.y * self.speed
            twist_stamped.twist.linear.z = self.z * self.speed
            twist_stamped.twist.angular.x = 0.0
            twist_stamped.twist.angular.y = 0.0
            twist_stamped.twist.angular.z = self.th * self.turn
            
            self.publisher.publish(twist_stamped)
            
            try:
                rate.sleep()
            except:
                break

    def run_teleop(self):
        """Main keyboard control loop - identical to working test script"""
        try:
            tty.setcbreak(sys.stdin.fileno())
            
            while self.running and rclpy.ok():
                # Check for keyboard input (same as working test script)
                ready, _, _ = select.select([sys.stdin], [], [], 0.1)
                
                if ready:
                    key = sys.stdin.read(1)
                    
                    if ord(key) == 3:  # Ctrl+C
                        print("\nCtrl+C detected, shutting down...")
                        break
                    elif ord(key) == 27:  # ESC key
                        print("\nESC pressed, shutting down...")
                        break
                    else:
                        self.process_key(key)
                
                # Let ROS2 process callbacks occasionally
                rclpy.spin_once(self, timeout_sec=0.001)
                
        except KeyboardInterrupt:
            print("\nKeyboard interrupt...")
        except Exception as e:
            print(f"\nError: {e}")
        finally:
            # Stop robot
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0
            self.th = 0.0
            self.running = False
            
            # Restore terminal
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def process_key(self, key):
        """Process individual key presses"""
        if key in moveBindings.keys():
            self.x = moveBindings[key][0]
            self.y = moveBindings[key][1]
            self.z = moveBindings[key][2]
            self.th = moveBindings[key][3]
            # print(f"Movement: {key} -> x:{self.x}, y:{self.y}, z:{self.z}, th:{self.th}")
            
        elif key in speedBindings.keys():
            self.speed = self.speed * speedBindings[key][0]
            self.turn = self.turn * speedBindings[key][1]
            self.status = self.status + 1
            
            print(vels(self.speed, self.turn))
            if self.status == 14:
                print(msg)
                self.status = 0
                
        else:
            # Unknown key - show what was pressed and stop
            print(f"Unknown key '{key}' (ASCII: {ord(key)}) - stopping robot")
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0
            self.th = 0.0


def main(args=None):
    # Initialize ROS2
    rclpy.init(args=args)
    
    try:
        # Create node
        teleop_node = TeleopTwistStampedKeyboard()
        
        # Run keyboard control (this blocks)
        teleop_node.run_teleop()
        
    except Exception as e:
        print(f"Error in main: {e}")
    
    finally:
        print("Cleaning up...")
        if 'teleop_node' in locals():
            teleop_node.running = False
            teleop_node.destroy_node()
        
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()