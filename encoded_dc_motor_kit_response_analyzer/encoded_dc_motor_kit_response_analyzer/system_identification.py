import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState  # Correct
from control.timeresp import step_info
import numpy as np
import matplotlib.pyplot as plt
import time
import re
import os
from ament_index_python.packages import get_package_share_directory

class SystemIdentification(Node):
    def __init__(self):
        super().__init__('system_identification')

        # Subscribers
        self.output_sub = self.create_subscription( JointState, '/joint_states', self.output_callback, 10)

        # Publishers
        self.reference_pub = self.create_publisher(Float64MultiArray, '/effort_controller/commands', 10)
        
        # Read the text file
        file_path = os.path.expanduser("~/robotics_inc/motor_kit_ws/src/motor-control-kit/encoded_dc_motor_kit_response_analyzer/encoded_dc_motor_kit_response_analyzer/prbs_signal_array_range_2.txt")

        with open(file_path, "r") as file:
            data = file.read()

        # Extract numbers from the text using regex
        self.numbers = list(map(int, re.findall(r'\d+', data)))

        # Convert to a Python list
        self.iterations = self.numbers[0] # Print to verify
        self.number_run = 1
        
        self.time_data = []
        self.input_data = []
        self.output_data = []
        
        self.xn_1 = [0.0, 0.0, 0.0, 0.0]  # Input history
        self.yn_1 = [0.0, 0.0, 0.0, 0.0]  # Output history

        self.input_coeffs = [0.009901, 0.009901, 0.0, 0.0]
        self.output_coeffs = [1.000000, 0.9802, 0.0, 0.0]
    
        
    def publish_with_subscriber_check(self, msg):
        """Wait for a subscriber and then attempt to publish the message multiple times."""
        max_attempts = 10
        attempt = 0

        while attempt < max_attempts:
            if self.reference_pub.get_subscription_count() > 0:  # Check if there is at least one subscriber
                self.reference_pub.publish(msg)
                self.get_logger().info(f"Publishing attempt {attempt + 1}/{max_attempts}...")
                return True  # Assume success if a subscriber exists when publishing
            else:
                self.get_logger().warn("No subscribers found. Retrying...")
                time.sleep(1)  # Wait a bit before retrying
                attempt += 1
            
        return False  # If max attempts are reached and no subscribers, return failure

    def output_callback(self, msg):
        """Callback to capture system output (filtered velocity)."""
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 
        self.time_data.append(current_time)
        current_prbs = self.numbers[self.number_run]
        prbs_msg = Float64MultiArray()
        
        self.input_data.append(self.numbers[self.number_run])
        
        if self.number_run == 1:
            # Publish the initial command
            self.get_logger().info(f"Publishing initial command:")
            # Ensure the message is published successfully
            success = self.publish_with_subscriber_check(prbs_msg)
            if success:
                self.get_logger().info(f"Initial command published successfully:")
                self.initial_command_sent = True
            else:
                self.get_logger().warn("Failed to publish initial command after multiple attempts.")
        else:
            prbs_msg.data = [current_prbs]
            self.reference_pub.publish(prbs_msg)
            
        self.number_run = self.number_run + 1
        self.output_data.append(self.filter_voltage_velocity_value(msg.velocity[0]))
        
        if self.number_run > self.iterations:
            calculate_and_print_metrics()
            

    
    def save_data_to_csv(self, time_array, response_array):
        """Save the time and response data to a CSV file."""
        with open('step_response_data.csv', 'w') as f:
            f.write("Time,Output\n")
            for time, output in zip(time_array, response_array):
                f.write(f"{time:.6f},{output:.6f}\n")
        self.get_logger().info("Data saved to step_response_data.csv.")


    def plot_data(self, time_array, response_array):
        """Plot the step response data."""
        plt.figure(figsize=(10, 6))
        plt.plot(time_array, response_array, label='Filtered Velocity')
        plt.axhline(y=3.0, color='r', linestyle='--', label='Step Input (3.0)')
        plt.xlabel('Time (s)')
        plt.ylabel('Velocity')
        plt.title('Step Response Analysis')
        plt.legend()
        plt.grid(True)
        plt.savefig('step_response_plot.png')  # Save the plot as an image
        plt.show()
        self.get_logger().info("Step response plot saved and displayed.")

    
    def calculate_and_print_metrics(self):
        """Calculate and print step response metrics."""
        if len(self.time_data) < 1000 or len(self.output_data) < 1000:
            #self.get_logger().info("Waiting for sufficient data...")
            return

        try:
            # Ensure time and response arrays are numpy arrays
            self.get_logger().info(f"Time array length: {len(self.time_data)} Response array length: {len(self.output_data)}")
            
            # Truncate the longer array to ensure they have the same length
            min_length = min(len(self.time_data), len(self.output_data))
            time_array = np.array(self.time_data[:min_length])
            response_array = np.array(self.output_data[:min_length])
            
            #subtract the start time from the time array
            time_array = time_array - time_array[0]
            
            #subtract the first output data from the data array
            #response_array = response_array - response_array[0] 

            # Calculate step response metrics
            metrics = step_info(response_array, time_array)

            # Print metrics to the console
            self.get_logger().info("Step Response Metrics:")
            for key, value in metrics.items():
                self.get_logger().info(f"{key}: {value}")
            

            # Save data to CSV
            self.save_data_to_csv(time_array, response_array)
            
            # Return the system to its initial state
            self.get_logger().info("Resetting the system...")
            step_value = 0.0  # Desired step reference velocity
            step_msg = Float64MultiArray()
            step_msg.data = [step_value]
            self.get_logger().info(f"Publishing step input: {step_value}")
            self.reference_pub.publish(step_msg)

            # Plot the data
            self.plot_data(time_array, response_array)

            # Kill the subscribers
            self.destroy_subscription(self.output_sub)
            self.get_logger().info("Subscribers killed.")

            # Stop calculating once metrics are printed
            rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f"Error calculating metrics: {e}")
    
            
    def filter_voltage_velocity_value(self, shaft_velocity):
        # Filtering the data
        self.yn_1[0] = (self.output_coeffs[1] * self.yn_1[1] + self.input_coeffs[0] * shaft_velocity + self.input_coeffs[1] * self.xn_1[1])

        # Shift values in history buffers
        self.xn_1[3] = self.xn_1[2]
        self.xn_1[2] = self.xn_1[1]
        self.xn_1[1] = shaft_velocity

        self.yn_1[3] = self.yn_1[2]
        self.yn_1[2] = self.yn_1[1]
        self.yn_1[1] = self.yn_1[0]

        return self.yn_1[0]


def main(args=None):
    rclpy.init(args=args)
    node = SystemIdentification()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()