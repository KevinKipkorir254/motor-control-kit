#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from std_msgs.msg import Char

import sys, select, termios, tty

# Save terminal settings to restore them later
settings = termios.tcgetattr(sys.stdin)

# Instructions and keybindings for controlling the robot
msg = """
                    Motor Control Commands
Reading from the keyboard and Publishing to /microROS/Control_keys!
----------------------------------------------------------------
.(>) - rotate clockwise
,(<) - rotate counterclockwise

k    - stop
w    - increase speed by 10%
x    - decrease speed by 10%
----------------------------------------------------------------

CTRL-C to quit
"""

class KeyboardMotorCommandsNode(Node):
    def __init__(self, name):
        super().__init__(name)

        self.get_logger().info(msg)
        self.publish_topic = "/microROS/Control_keys"

        self.commands_publisher = self.create_publisher(Char, self.publish_topic, qos_profile_system_default)

        self.timer_period = 1.0

        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    
    # Function to read a single key from the console
    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        key = ord(key)
        return key


    # Timer callback function
    def timer_callback(self):
        char_msg = Char()
        char_msg.data = self.get_key()
        self.commands_publisher.publish(char_msg)



def main(args = None):
    if args is None:
        args = sys.argv[1:]
    # Initialize ros2 communication
    rclpy.init()
    node = KeyboardMotorCommandsNode("keyboard_motor_commands_node")
    rclpy.spin(node)
 
    node.destroy_node()
    rclpy.shutdown()

    
    
# Entry point of the script
if __name__ == '__main__':
    main()
