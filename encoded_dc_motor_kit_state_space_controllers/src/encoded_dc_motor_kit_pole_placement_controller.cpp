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
        previous_time = this->get_clock()->now();
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
        control_output = clamp_controller_value(control_output);
        RCLCPP_INFO(this->get_logger(), "Control: '%.4f'", control_output);

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
        double yn = 0.969 * yn_1 + 0.0155 * shaft_velocity + 0.0155 * xn_1;
        xn_1 = shaft_velocity;
        yn_1 = yn;
        return yn;
    }

    double update_control_value(double shaft_velocity)
    {

        double error = reference_velocity - shaft_velocity; // error = r - y

        // replacing the values

        // Inside your loop:
        rclcpp::Time current_time = this->get_clock()->now();
        rclcpp::Duration period = current_time - previous_time;
        previous_time = current_time;

        state_space[0] = shaft_velocity;
        state_space[1] = shaft_velocity - yn_1;
        state_space_total = ((state_space[0] * gains_[0]) + (state_space[1] * gains_[1]));

        error_integral += error * period.seconds();
        double Khat_result = error_integral * gains_[2];

        double controller_output = Khat_result - state_space_total;

        return controller_output;
    }

    double clamp_controller_value(double value)
    {
        if (value > 255)
        {
            return 255;
        }
        else if (value < -255)
        {
            return -255;
        }
        else
        {
            return value;
        }
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

    rclcpp::Time previous_time;
    /*---------------------CONTROLLER INIT VALUES-------------------------*/
    // DATA STORAGE
    double state_space[2] = {0.0, 0.0};
    double state_space_total = 0.0;
    double gains_[3] = {-25.0464, -14.3974, 26.4901};
    double error_integral = 0.0;
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