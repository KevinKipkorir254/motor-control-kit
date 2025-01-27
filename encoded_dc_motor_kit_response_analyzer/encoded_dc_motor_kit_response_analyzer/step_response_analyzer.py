import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from control.timeresp import step_info
import numpy as np

class StepResponseAnalyzer(Node):
    def __init__(self):
        super().__init__('step_response_analyzer')

        # Subscribers
        self.output_sub = self.create_subscription(Float64MultiArray, '/filtered_velocity', self.output_callback, 10)

        # Publishers
        self.reference_pub = self.create_publisher(Float64MultiArray, '/velocity/commands', 10)

        # Data storage
        self.reference_data = []  # Input data (reference velocity)
        self.output_data = []     # Output data (filtered velocity)
        self.time_data = []       # Time data

        # Timing
        self.start_time = self.get_clock().now().nanoseconds * 1e-9
        self.sample_rate = 0.01   # Sampling interval (10ms)
        self.step_set = False     # Flag to ensure the step input is sent once

        # Timers
        self.create_timer(self.sample_rate, self.generate_step_input)
        self.create_timer(self.sample_rate, self.calculate_and_print_metrics)

    def generate_step_input(self):
        """Generate a step input and publish it to the reference_velocity topic."""
        if not self.step_set:
            self.get_logger().info("Generating step input...")
            step_value = 1.0  # Desired step reference velocity
            step_msg = Float64MultiArray()
            step_msg.data = [step_value]

            # Publish the step input
            # print before piblishing
            self.get_logger().info(f"Publishing step input: {step_value}")
            self.reference_pub.publish(step_msg)

            # Store the reference data for analysis
            current_time = self.get_clock().now().nanoseconds * 1e-9 - self.start_time
            self.time_data.append(current_time)
            self.reference_data.append(step_value)

            self.step_set = True

    def output_callback(self, msg):
        """Callback to capture system output (filtered velocity)."""
        current_time = self.get_clock().now().nanoseconds * 1e-9 - self.start_time
        self.output_data.append(msg.data)
        self.time_data.append(current_time)

    def calculate_and_print_metrics(self):
        """Calculate and print step response metrics."""
        if len(self.time_data) < 1000 or len(self.output_data) < 1000:
            self.get_logger().info("Waiting for sufficient data...")
            return

        try:
            # Ensure time and response arrays are numpy arrays
            #print length for time array and response array
            self.get_logger().info(f"Time array length: {len(self.time_data)} Response array length: {len(self.output_data)}")
            
            #pop the last element from time array
            self.time_data.pop()
            #self.reference_pub.destroy()
            self.output_sub.destroy()
            time_array = np.array(self.time_data)
            response_array = np.array(self.output_data)

            # Calculate step response metrics
            metrics = step_info(time_array, response_array)

            # Print metrics to the console
            self.get_logger().info("Step Response Metrics:")
            for key, value in metrics.items():
                self.get_logger().info(f"{key}: {value}")
                
            #end the other subscriptions and topics
            self.get_logger().info("Step response analysis complete.")
            
            # Output the data to a CSV file with 4 decimal places
            with open('step_response_data.csv', 'w') as f:
              f.write("Time,Output\n")
              max_length = min(len(self.time_data), len(self.output_data))
              for i in range(min(1000, max_length)):  # Ensures we stop at 1000 or the shortest list length
                # Convert to float if needed and format to 4 decimal places
                time = (self.time_data[i]) 
                output = (self.output_data[i]) 
                f.write(f"{time:.6f},{output[0]:.6f}\n")

            # Stop calculating once metrics are printed
            rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f"Error calculating metrics: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = StepResponseAnalyzer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

        
