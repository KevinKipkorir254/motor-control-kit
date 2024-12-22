#!/usr/bin/env python3
"""
ROS 2 Node for Plotting System Response

This node subscribes to a ROS 2 topic where the feedback from encoders is published.
The feedback is used as the system output, and the time is obtained from the timestamp on the message.
The response curve is plotted by plotting system output against time.

Author: [Your Name]
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt

class ResponsePlotterNode(Node):
    def __init__(self):
        super().__init__('response_plotter_node')
        self.system_output = []
        self.time = []

        self.subscription = self.create_subscription(
            Float64MultiArray,
            "/position_controller/commands",  # Update with your actual topic
            self.feedback_callback,
            10
        )

        self.subscription  # Prevent unused variable warning

    def feedback_callback(self, msg):
        self.system_output.append(msg.data)
        self.time.append(self.get_clock().now().nanoseconds / 1e9)  # Use ROS 2 clock for time stamps

    def plot_response(self):
        plt.plot(self.time, self.system_output)
        plt.xlabel('Time')
        plt.ylabel('System Output')
        plt.title('System Response')

        # Add minor grids
        plt.grid(True, which='both')
        plt.minorticks_on()
        plt.grid(True, which='minor', linestyle=':', linewidth='0.5', color='gray')

        # Calculate system characterization parameters
        rise_time = self.calculate_rise_time()
        settling_time = self.calculate_settling_time()
        overshoot = self.calculate_overshoot()
        settling_criteria = self.calculate_settling_criteria()

        # Add text annotations for system characterization parameters
        plt.text(0.1, 0.9, f'Rise Time: {rise_time:.2f}', transform=plt.gca().transAxes)
        plt.text(0.1, 0.85, f'Settling Time: {settling_time:.2f}', transform=plt.gca().transAxes)
        plt.text(0.1, 0.8, f'Overshoot: {overshoot:.2f}', transform=plt.gca().transAxes)
        plt.text(0.1, 0.75, f'Settling Criterion: {settling_criteria}', transform=plt.gca().transAxes)

        # Mark points of interest on the curve
        plt.scatter([rise_time, settling_time], [self.system_output[self.time.index(rise_time)], self.system_output[self.time.index(settling_time)]], color='red')
        plt.annotate('Rise Time', xy=(rise_time, self.system_output[self.time.index(rise_time)]), xytext=(rise_time, self.system_output[self.time.index(rise_time)] + 0.5),
                     arrowprops=dict(facecolor='black', arrowstyle='->'), horizontalalignment='left', verticalalignment='bottom')
        plt.annotate('Settling Time', xy=(settling_time, self.system_output[self.time.index(settling_time)]), xytext=(settling_time, self.system_output[self.time.index(settling_time)] + 0.5),
                     arrowprops=dict(facecolor='black', arrowstyle='->'), horizontalalignment='left', verticalalignment='bottom')

        plt.show()

    def calculate_rise_time(self):
        # Rise time: Time taken for the response to go from 10% to 90% of the final value
        ten_percent = 0.1 * (max(self.system_output) - min(self.system_output)) + min(self.system_output)
        ninety_percent = 0.9 * (max(self.system_output) - min(self.system_output)) + min(self.system_output)
        for i, output in enumerate(self.system_output):
            if output > ten_percent:
                rise_time_index = i
                break
        for i, output in enumerate(self.system_output):
            if output > ninety_percent:
                rise_time_index_ninety = i
                break
        rise_time = self.time[rise_time_index_ninety] - self.time[rise_time_index]
        return rise_time

    def calculate_settling_time(self):
        # Settling time: Time taken for the response to settle within a certain range of the final value
        final_value = self.system_output[-1]
        settling_range = 0.02 * final_value  # Settling within 2% of the final value
        for i, output in enumerate(reversed(self.system_output)):
            if abs(output - final_value) > settling_range:
                settling_time_index = len(self.system_output) - i
                break
        settling_time = self.time[settling_time_index]
        return settling_time

    def calculate_overshoot(self):
        # Overshoot: Maximum percentage by which the response exceeds the final value
        max_output = max(self.system_output)
        final_value = self.system_output[-1]
        overshoot = (max_output - final_value) / final_value * 100
        return overshoot

    def calculate_settling_criteria(self):
        # Settling Criterion: Determines the range within which the response settles
        settling_range = 0.02 * self.system_output[-1]  # 2% settling range
        for output in self.system_output[-50:]:  # Check last 50 data points
            if abs(output - self.system_output[-1]) > settling_range:
                return "Fail"
        return "Pass"

def main():
    rclpy.init()
    node = ResponsePlotterNode()
    node.plot_response()

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
