#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <string>
#include <iostream>
#include <cstdlib> // for system()

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class LeadLagCompensator : public rclcpp::Node
{
public:
    LeadLagCompensator()
        : Node("LeadLagCompensator"), count_(0)
    {
        // Text to display using figlet
        std::string text = "MOTOR-KIT";
        // Construct the figlet command with the -c option for centering
        std::string command = "figlet -w $(tput cols) -c \"" + text + "\" | lolcat"; //added lolcat
        // Execute the command
        std::system(command.c_str());
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/effort_controller/commands", 10);
        filtered_velocity_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/filtered_velocity", 10);
        subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("/velocity/commands", 10, std::bind(&LeadLagCompensator::update_reference_velocity, this, std::placeholders::_1));
        joint_state_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&LeadLagCompensator::update_shaft_state_and_control_value, this, std::placeholders::_1));
        lead_lag_compensator_states = this->create_publisher<std_msgs::msg::Float64MultiArray>("/lead_lag_compensator_states", 10);

        // Configure layout only once in the constructor
        layout_.dim.resize(1);
        layout_.dim[0].label = "LeadLagCompensatorStates";
        layout_.dim[0].size = 6; // Number of elements in the array
        layout_.dim[0].stride = 1;

        labels_ = {"y_gc_0", "G_c_output_1 * y_gc_1", "G_c_output_2 * y_gc_2",
                   "G_c_input_0 * u_gc_0", "G_c_input_1 * u_gc_1", "G_c_input_2 * u_gc_2"};

        for (int i = 0; i < 3; ++i)
        {
            u_gc[i] = 0.0;
            y_gc[i] = 0.0;
        }
    }

private:
    void timer_callback()
    {
        auto message = std::make_shared<std_msgs::msg::Float64MultiArray>();
        message->data.push_back(0.0);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%.4f'", 0.0);
        publisher_->publish(*message);
    }

    void update_shaft_state_and_control_value(const sensor_msgs::msg::JointState &msg)
    {
        // get shaft position
        auto shaft_it = std::find(msg.name.begin(), msg.name.end(), "shaft_joint");
        if (shaft_it != msg.name.end())
        {
            int shaft_index = std::distance(msg.name.begin(), shaft_it);
            shaft_position_ = msg.position[shaft_index];
            shaft_velocity_ = msg.velocity[shaft_index];
            // RCLCPP_INFO(this->get_logger(), "Pos: '%f', Vel: '%f'", shaft_position_, shaft_velocity_);

            // Instead of using RCLCPP_INFO (which prints a new line each time),
            // use std::cout with carriage return to update on the same line.
            static int spinner_index = 0;
            const std::string spinner = "|/-\\";
            std::cout << "\rROS Node is running... " << spinner[spinner_index] << std::flush;
            spinner_index = (spinner_index + 1) % spinner.size();
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Shaft not found");
            return;
        }
        // filter velocity data
        shaft_velocity_ = filter_voltage_velocity_value(shaft_velocity_);
        publish_filtered_velocity_value(shaft_velocity_);

        // get control value
        double control_output = update_control_value(shaft_velocity_);

        // publish control value
        publish_control_value(control_output);
    }

    void publish_control_value(double control_output)
    {
        auto control_message = std_msgs::msg::Float64MultiArray();
        control_message.data.push_back(control_output);
        publisher_->publish(control_message);
    }

    void publish_filtered_velocity_value(double filtered_velocity)
    {
        auto filtered_velocity_message = sensor_msgs::msg::JointState();
        filtered_velocity_message.header.stamp = this->now();  // Assign ROS timestamp
        filtered_velocity_message.velocity.push_back(filtered_velocity);
        filtered_velocity_publisher_->publish(filtered_velocity_message);
    }

    double filter_voltage_velocity_value(double shaft_velocity)
    {
        // OBTAINING THE DATA FEEDBACK
        // filtering the data
        double input_coeffs[4] = {0.009901, 0.009901, 0.0, 0.0};
        double output_coeffs[4] = {1.000000, 0.9802, 0.0, 0.0};
        yn_1[0] = output_coeffs[1] * yn_1[1] + input_coeffs[0] * shaft_velocity + input_coeffs[1] * xn_1[1];

        xn_1[3] = xn_1[2];
        xn_1[2] = xn_1[1];
        xn_1[1] = shaft_velocity;

        yn_1[3] = yn_1[2];
        yn_1[2] = yn_1[1];
        yn_1[1] = yn_1[0];
        return yn_1[0];
    }

    double update_control_value(double shaft_velocity)
    {
        // G_C COMPENSATOR INITIALISATION
        //G_c_output[3]    G_c_input[3]
        double G_c_input[3] = {gains_ * 1.000000000000001,gains_ * -1.676778026159205,gains_ * 0.677060853511581};
        double G_c_output[3] = {1.000000000000000, 1.636899746327462, -0.636932814955440};

        double error = reference_velocity - shaft_velocity; // error = r - y
        y_gc[0] = G_c_output[1] * y_gc[1] + G_c_output[2] * y_gc[2] + G_c_input[0] * error + G_c_input[1] * u_gc[1] + G_c_input[2] * u_gc[2];

        // update previous outputs
        y_gc[2] = y_gc[1];
        y_gc[1] = y_gc[0];

        // update previous inputs
        u_gc[2] = u_gc[1];
        u_gc[1] = error;

        // publish y_gc, G_c_output[1] * y_gc[1], G_c_output[2] * y_gc[2], G_c_input[0] * u_gc[0], G_c_input[1] * u_gc[1], G_c_input[2] * u_gc[2]
        auto lead_lag_compensator_states_message = std_msgs::msg::Float64MultiArray();
        lead_lag_compensator_states_message.data.push_back(error);
        lead_lag_compensator_states_message.data.push_back(y_gc[0]);
        lead_lag_compensator_states_message.data.push_back(G_c_output[1] * y_gc[1]);
        lead_lag_compensator_states_message.data.push_back(G_c_output[2] * y_gc[2]);
        lead_lag_compensator_states_message.data.push_back(G_c_input[0] * error);
        lead_lag_compensator_states_message.data.push_back(G_c_input[1] * u_gc[1]);
        lead_lag_compensator_states_message.data.push_back(G_c_input[2] * u_gc[2]);
        lead_lag_compensator_states->publish(lead_lag_compensator_states_message);

        return y_gc[0];
    }

    void update_reference_velocity(const std_msgs::msg::Float64MultiArray &msg)
    {
        reference_velocity = msg.data[0];
        // RCLCPP_INFO(this->get_logger(), "Ref: '%f'", reference_velocity);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr filtered_velocity_publisher_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr lead_lag_compensator_states;
    volatile double reference_velocity = 0.0;
    volatile double shaft_position_ = 0.0;
    volatile double shaft_velocity_ = 0.0;
    volatile double yn_1[4] = {0.0, 0.0, 0.0, 0.0};
    volatile double xn_1[4] = {0.0, 0.0, 0.0, 0.0};

    size_t count_;
    std_msgs::msg::MultiArrayLayout layout_;
    std::vector<std::string> labels_; // For debugging/logging purposes

    const double gains_ = 100;

    // THE INPUT OUPTU DATA
    double u_gc[3] = {0.0, 0.0, 0.0};
    double y_gc[3] = {0.0, 0.0, 0.0};
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LeadLagCompensator>());
    rclcpp::shutdown();
    return 0;
}