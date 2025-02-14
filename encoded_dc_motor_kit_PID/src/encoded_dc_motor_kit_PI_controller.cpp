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

        double error = reference_velocity - shaft_velocity; // error = r - y

        // computing the controller values
        // replacing the values
        controller_input[0] = error;
        controller_output[0] = output_coeffs[1] * controller_output[1] + input_coeffs[0] * controller_input[0] + input_coeffs[1] * controller_input[1];
        controller_input[1] = controller_input[0];
        controller_output[1] = controller_output[0];

        return controller_output[0];
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
    volatile double yn_1[4] = {0.0, 0.0, 0.0, 0.0};
    volatile double xn_1[4] = {0.0, 0.0, 0.0, 0.0};
    /*---------------------CONTROLLER INIT VALUES-------------------------*/
    // DATA STORAGE
    double controller_input[3] = {0.0, 0.0, 0.0};  // u[k], u[k-1], u[k-2]
    double controller_output[3] = {0.0, 0.0, 0.0}; // y[k], y[k-1], y[k-2]

    // coefficient storage
    double output_coeffs[2] = {1.0, 1.0};     // output coeffs y[k], y[k-1]
    double input_coeffs[2] = {205.3, -205.2}; // input coeffs, u[k], u[k-1]
    /*---------------------CONTROLLER INIT VALUES-------------------------*/

    size_t count_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LeadCompensator>());
    rclcpp::shutdown();
    return 0;
}