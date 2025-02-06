#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class LeadCompensator : public rclcpp::Node
{
public:
    LeadCompensator()
        : Node("LeadCompensator"), count_(0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/effort_controller/commands", 10);
        filtered_velocity_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/filtered_velocity", 10);
        subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("/velocity/commands", 10, std::bind(&LeadCompensator::update_reference_velocity, this, std::placeholders::_1));
        joint_state_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&LeadCompensator::update_shaft_state_and_control_value, this, std::placeholders::_1));
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
            RCLCPP_INFO(this->get_logger(), "Pos: '%f', Vel: '%f'", shaft_position_, shaft_velocity_);
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
        auto filtered_velocity_message = std_msgs::msg::Float64MultiArray();
        filtered_velocity_message.data.push_back(filtered_velocity);
        filtered_velocity_publisher_->publish(filtered_velocity_message);
    }

    double filter_voltage_velocity_value(double shaft_velocity)
    {
        // OBTAINING THE DATA FEEDBACK

        // update input
        filter_input[7] = filter_input[6];
        filter_input[6] = filter_input[5];
        filter_input[5] = filter_input[4];
        filter_input[4] = filter_input[3];
        filter_input[3] = filter_input[2];
        filter_input[2] = filter_input[1];
        filter_input[1] = shaft_velocity;

        // filtering the data
        filter_output[0] = 
        a_i[1] * filter_output[1] +
        a_i[2] * filter_output[2] + 
        a_i[3] * filter_output[3] +
        a_i[4] * filter_output[4] + 
        a_i[5] * filter_output[5] +
        a_i[6] * filter_output[6] +
        a_i[7] * filter_output[7] +
        b_i[0] * shaft_velocity + 
        b_i[1] * filter_input[1] + 
        b_i[2] * filter_input[2] +
        b_i[3] * filter_input[3] +
        b_i[4] * filter_input[4] +
        b_i[5] * filter_input[5] +
        b_i[6] * filter_input[6] +
        b_i[7] * filter_input[7];

        // update outputs
        filter_output[7] = filter_output[6];
        filter_output[6] = filter_output[5];
        filter_output[5] = filter_output[4];
        filter_output[4] = filter_output[3];
        filter_output[3] = filter_output[2];
        filter_output[2] = filter_output[1];
        filter_output[1] = filter_output[0];

        return filter_output[0];
    }

    double update_control_value(double shaft_velocity)
    {
        // G_C COMPENSATOR INITIALISATION
        double G_c_output[2] = {0.00, 0.9999089};
        double G_c_input[2] = {170.904, -170.754};

        double error = reference_velocity - shaft_velocity; // error = r - y
        y_gc[0] = G_c_output[1] * y_gc[1] + G_c_input[0] * error + G_c_input[1] * u_gc[1];

        /// update previus error
        u_gc[1] = error;

        // update previous output
        y_gc[1] = y_gc[0];

        return y_gc[0];
    }

    void update_reference_velocity(const std_msgs::msg::Float64MultiArray &msg)
    {
        reference_velocity = msg.data[0];
        RCLCPP_INFO(this->get_logger(), "Ref: '%f'", reference_velocity);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr filtered_velocity_publisher_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
    volatile double reference_velocity;
    volatile double shaft_position_ = 0.0;
    volatile double shaft_velocity_ = 0.0;
    volatile double yn_1 = 0.0, xn_1 = 0.0;
    size_t count_;

    // filter input and output parameters
    double a_i[7] = {3.65428118, -5.86934257, 5.21922855, -2.68936399, 0.75741618, -0.0907553};
    double b_i[8] = {0.00028962, 0.00173775, 0.00434436, 0.00579249, 0.00434436, 0.00173775,
                     0.00028962};
    double filter_input[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double filter_output[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // THE INPUT OUPTU DATA
    double u_gc[3] = {0.0, 0.0, 0.0};
    double y_gc[3] = {0.0, 0.0, 0.0};
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LeadCompensator>());
    rclcpp::shutdown();
    return 0;
}