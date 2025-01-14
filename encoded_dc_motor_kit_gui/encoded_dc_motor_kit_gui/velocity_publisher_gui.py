#!/usr/bin/env python3

""" 
    A node to publish velocity to the hardware interface  
    Provides a simple GUI interface
"""
import tkinter as tk
from rclpy.node import Node
from rclpy import init, spin, shutdown
from std_msgs.msg import Float64MultiArray
import threading


class EffortControllerPublisher(Node):
    def __init__(self):
        super().__init__("velocity_controller_gui")
        self.publisher = self.create_publisher(Float64MultiArray, "/velocity/commands", 10)

    def publish_effort(self, effort_value):
        msg = Float64MultiArray()
        msg.data = [float(effort_value)]  # Convert to float
        self.publisher.publish(msg)
        self.get_logger().info(f"Published: {effort_value}")


class EffortControlGUI:
    def __init__(self, ros_node):
        self.node = ros_node

        # Create the GUI window
        self.root = tk.Tk()
        self.root.title("Velocity Controller GUI")

        # Slider for setting effort value
        self.slider = tk.Scale(
            self.root,
            from_=-10.0,
            to=10.0,
            orient="horizontal",
            label="Velocity Value",
            length=300,
            resolution=1,  # Set step size to 0.1
            command=self.publish_value,  # Publish whenever slider changes
        )
        self.slider.pack(pady=20)

        # Bind the release event to spring back the slider
        # self.slider.bind("<ButtonRelease-1>", self.spring_back)

        # Exit button
        self.exit_button = tk.Button(
            self.root,
            text="Exit",
            command=self.exit_gui,
            width=20,
            height=2,
        )
        self.exit_button.pack(pady=10)

    def publish_value(self, value):
        # Convert the slider value to an integer and publish
        effort_value = value
        self.node.publish_effort(effort_value)

    def spring_back(self, event):
        # Reset the slider value to 0 (or a safe default value)
        self.slider.set(0)  # You can change this to another value if needed
        self.node.publish_effort(0)  # Optionally publish 0 when the slider springs back

    def exit_gui(self):
        self.node.destroy_node()
        shutdown()
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def main():
    init()
    ros_node = EffortControllerPublisher()

    # Run ROS 2 spin in a separate thread
    ros_thread = threading.Thread(target=spin, args=(ros_node,), daemon=True)
    ros_thread.start()

    # Run the GUI
    gui = EffortControlGUI(ros_node)
    gui.run()


if __name__ == "__main__":
    main()
