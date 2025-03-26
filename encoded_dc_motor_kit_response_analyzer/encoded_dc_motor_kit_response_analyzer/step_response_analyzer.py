import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState  # Correct
from control.timeresp import step_info
import numpy as np
import matplotlib.pyplot as plt
import time

class StepResponseAnalyzer(Node):
    def __init__(self):
        super().__init__('step_response_analyzer')

        # Subscribers
        self.output_sub = self.create_subscription( JointState, '/filtered_velocity', self.output_callback, 10)

        # Publishers
        self.reference_pub = self.create_publisher(Float64MultiArray, '/velocity/commands', 10)

        # Data storage
        self.reference_data = []  # Input data (reference velocity)
        self.output_data = []     # Output data (filtered velocity)
        self.time_data = []       # Time data
        self.time_data_finisher = []

        # Timing
        self.start_time = self.get_clock().now().nanoseconds * 1e-9
        self.sample_rate = 1/100  # Sampling interval (10ms)
        self.step_set = False     # Flag to ensure the step input is sent once
        self.initial_command_sent = False  # Flag to ensure initial command is sent once
        self.recording_started = False  # Flag to start recording data

        # Timers
        self.create_timer(self.sample_rate, self.generate_initial_command)
        self.create_timer(self.sample_rate, self.generate_step_input)
        self.create_timer(self.sample_rate, self.calculate_and_print_metrics)
    
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

    def generate_initial_command(self):
        """Generate an initial velocity command of 2."""
        if not self.initial_command_sent:
            self.get_logger().info("Generating initial command...")
            initial_value = 2.0  # Initial velocity command
            initial_msg = Float64MultiArray()
            initial_msg.data = [initial_value]

            # Publish the initial command
            self.get_logger().info(f"Publishing initial command: {initial_value}")

            # Ensure the message is published successfully
            success = self.publish_with_subscriber_check(initial_msg)

            if success:
                self.get_logger().info(f"Initial command published successfully: {initial_value}")
                self.initial_command_sent = True

                # Start a timer to send the step input after 4 seconds
                time.sleep(4)
                self.create_timer(1.0, self.enable_step_input)
            else:
                self.get_logger().warn("Failed to publish initial command after multiple attempts.")


    def enable_step_input(self):
        """Enable the step input after 4 seconds."""
        self.get_logger().info("Enabling step input...")
        self.step_set = False

    def generate_step_input(self):
        """Generate a step input and publish it to the reference_velocity topic."""
        if not self.step_set and self.initial_command_sent:
            self.get_logger().info("Generating step input...")
            step_value = 3.0  # Desired step reference velocity
            step_msg = Float64MultiArray()
            step_msg.data = [step_value]

            # Publish the step input
            self.get_logger().info(f"Publishing step input: {step_value}")
            self.reference_pub.publish(step_msg)

            # Store the reference data for analysis
            self.reference_data.append(step_value)

            self.step_set = True
            self.recording_started = True

            # Start a timer to wait for 10 seconds before calculating metrics
            self.create_timer(10.0, self.enable_metrics_calculation)

    def output_callback(self, msg):
        """Callback to capture system output (filtered velocity)."""
        #self.get_logger().info("Callback triggered!")  
        if self.recording_started:
            #self.get_logger().info("Callback triggered 22!")  
            current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9  # Use ROS 2 timestamp
            #current_time_finisher = self.get_clock().now().nanoseconds * 1e-9 - self.start_time
            #self.time_data_finisher.append(current_time_finisher)
            self.time_data.append(current_time)
            self.output_data.append(msg.velocity[0])  # Ensure we only store the first element of the array
            #if hasattr(msg, "velocity"):  # Ensure msg has the velocity attribute
            #    self.get_logger().info(f"Raw velocity message: {msg.velocity}")
            #    if isinstance(msg.velocity, list) or isinstance(msg.velocity, tuple):
            #        self.output_data.append(msg.velocity[0])  # Store first element
            #        self.get_logger().info(f"Velocity stored: {msg.velocity[0]}")
            #    else:
            #        self.output_data.append(msg.velocity)  # Direct storage
            #        self.get_logger().info(f"Velocity stored: {msg.velocity}")
            #else:
            #    self.get_logger().error("Error: msg.velocity does not exist!")
            

    def enable_metrics_calculation(self):
        """Enable metrics calculation after 10 seconds."""
        #self.get_logger().info("Enabling metrics calculation...")
        self.calculate_and_print_metrics()

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