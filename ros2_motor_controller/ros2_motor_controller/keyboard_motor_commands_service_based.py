#!/usr/bin/env python3
import sys, termios, select, tty
import rclpy
from rclpy.node import Node
from functools import partial
from ros2_motor_controller_msgs.srv import KeyboardCommands

# Save terminal settings to restore them later
settings = termios.tcgetattr(sys.stdin)

class KeyboardMotorCommandsClient(Node):
    def __init__(self, name):
        super().__init__(name)

        self.client_ = self.create_client(KeyboardCommands, "/microROS/KeyboardMotorCommands")

        # Wait until server is connected
        while not self.client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting ... ")
        
        self.req = KeyboardCommands.Request()

    def send_request(self, request):
        self.req.keyboard_input = request
        self.future = self.client_.call_async(self.req)
        self.future = self.future.add_done_callback(partial(self.response_callback))

    # Get value of the pressed key
    # Function to read a single key from the console
    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        key = ord(key)
        self.send_request(key)

    # Callback function for the service client
    def response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Received joint angles: {response.joint_angles}")
        except Exception as e:
            self.get_logger().error(f"Service call failed {e}")

def main(args = None):
    rclpy.init()
    node = KeyboardMotorCommandsClient("keyboard_motor_commands_client");
    rclpy.spin(node)
    rclpy.shutdown()


# Entry point
if __name__ == "__main__":
    main()