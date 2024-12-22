#!/usr/bin/env python3

""" 
    A node to subscribe to the topic where feedback from the encoders is published
    It uses the data obtained to run the digital twin by publishing it to the joint states.
    The data received is first convertered to radians
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import Float64MultiArray

import numpy as np

# Constants
ENCODER_PULSES_PER_REVOLUTION: int = 500

class FeedbackSubscriberNode(Node):
    def __init__(self):
        super().__init__('feedback_subscriber_node')

        # Variable to hold feedback value in radians
        self.feedback = []

        # subscriber to /position_feedback topic
        self.subscription = self.create_subscription(
            Int32,
            '/position_feedback',  
            self.feedback_callback,
            10)
        self.subscription  # prevent unused variable warning

        # publish topic
        self.publish_topic = "/position_controller/commands"
        # publisher
        self.angular_position_pub = self.create_publisher(Float64MultiArray, self.publish_topic, 10)

        # Publishing frequency 
        self.publish_freq = 60
        self.timer = self.create_timer(1/self.publish_freq, self.publish_joint_position)

    def feedback_callback(self, msg):
        # Convert encoder feedback to radians
        feedback_in_radians = self.convert_to_radians(msg.data)
        self.feedback = feedback_in_radians

    def convert_to_radians(self, feedback):
        feedback = (2 * np.pi * feedback/ENCODER_PULSES_PER_REVOLUTION)
        feedback = [feedback]
        return feedback

    def publish_joint_position(self):
        # Publish joint position to the position controller topic
        joint_position = Float64MultiArray()
        joint_position.data = self.feedback
        self.angular_position_pub.publish(joint_position)

        # Replace the below code with actual publishing code for your joint states topic
        self.get_logger().info(f'Publishing feedback joint position: {self.feedback}')

def main(args=None):
    rclpy.init(args=args)
    feedback_subscriber_node = FeedbackSubscriberNode()
    rclpy.spin(feedback_subscriber_node)
    feedback_subscriber_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
