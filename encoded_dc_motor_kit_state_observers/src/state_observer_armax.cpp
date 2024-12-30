#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
    MinimalSubscriber()
        : Node("state_observer_armax")
    {
        joint_state_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10, std::bind(&MinimalSubscriber::joint_state_callback, this, _1));
        state_observer_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "state_observer", 10);
        subscription_input_voltage_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("/effort_controller/commands", 10, std::bind(&MinimalSubscriber::update_input, this, std::placeholders::_1));
        A_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("A_publisher", 10);
        B_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("B_publisher", 10);
        K_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("K_publisher", 10);
        A_B_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("A_B_publisher", 10);

        A_ << 0.0, 1.0, -34.91, -13.87;
        B_ << 0.0, 0.755;

        C_ << 1.0, 0.0;
        k_e_ << 26.0, 63.0;
        x_hat << 0.0, 0.0;
    }

    void joint_state_callback(const sensor_msgs::msg::JointState &msg)
    {
        // get shaft position
        auto shaft_it = std::find(msg.name.begin(), msg.name.end(), "shaft_joint");
        if (shaft_it != msg.name.end())
        {
            int shaft_index = std::distance(msg.name.begin(), shaft_it);
            shaft_position_ = msg.position[shaft_index];
            unfiltered_shaft_velocity_ = msg.velocity[shaft_index];

            // put your main code here, to run repeatedly:
            // OBTAINING THE DATA FEEDBACK
            // filtering the data
            shaft_velocity_ = 0.969 * yn_1 + 0.0155 * unfiltered_shaft_velocity_ + 0.0155 * xn_1; // low pass filter
            xn_1 = unfiltered_shaft_velocity_;
            yn_1 = shaft_velocity_;

            shaft_acceleration_ = (shaft_velocity_ - shaft_previous_acceleration_) / 0.01;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Shaft not found");
            return;
        }

        y_outputs_ << shaft_velocity_, shaft_acceleration_;
        state_observer();
        publisher_callback();
    }

    void update_input(const std_msgs::msg::Float64MultiArray &msg)
    {
        input_voltage_ = msg.data[0];
    }

    void state_observer()
    {
        // state observer

        // solve for A
        Eigen::Vector2d A_result = (A_ * x_hat);

        // publish a result
        auto A_message = std_msgs::msg::Float64MultiArray();
        A_message.data.push_back(A_result(0));
        A_message.data.push_back(A_result(1));
        A_publisher_->publish(A_message);

        // Solve for B
        Eigen::Vector2d B_result = (B_ * input_voltage_);

        // publis b result
        auto B_message = std_msgs::msg::Float64MultiArray();
        B_message.data.push_back(B_result(0));
        B_message.data.push_back(B_result(1));
        B_publisher_->publish(B_message);

        Eigen::Vector2d A_B = A_result + B_result;

        // publish A_B result
        auto A_B_message = std_msgs::msg::Float64MultiArray();
        A_B_message.data.push_back(A_B(0));
        A_B_message.data.push_back(A_B(1));
        A_B_publisher_->publish(A_B_message);

        // solve for C
        double y_hat = C_ * x_hat;

        // publish as  k result
        auto K_message = std_msgs::msg::Float64MultiArray();
        K_message.data.push_back(y_hat);
        K_publisher_->publish(K_message);

        // using the ks
        Eigen::Vector2d k_result = k_e_ * (y_outputs_(0) - y_hat);

        // //update state estimate
        double dt_ = 0.01;
        Eigen::Vector2d x_hat_dot = (A_B + k_result);

        x_hat += x_hat_dot * dt_;
    }

    void publisher_callback()
    {
        auto message = std_msgs::msg::Float64MultiArray();
        message.data.push_back(x_hat(0));
        message.data.push_back(x_hat(1));
        state_observer_publisher_->publish(message);
    }

private:
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_input_voltage_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr state_observer_publisher_; // effort controller publisher
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr A_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr B_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr K_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr A_B_publisher_;

    Eigen::Matrix2d A_;
    Eigen::Vector2d B_;
    Eigen::Matrix<double, 1, 2> C_;
    Eigen::Vector2d y_outputs_;
    Eigen::Vector2d k_e_;
    Eigen::Vector2d x_hat;
    // Eigen::Vector2d dt_;

    double shaft_position_ = 0.0;
    double shaft_velocity_ = 0.0;
    double shaft_acceleration_ = 0.0;
    double shaft_previous_acceleration_ = 0.0;
    double input_voltage_ = 0.0;
    double dt_;

    // Low pass filter data
    double unfiltered_shaft_velocity_ = 0.0;
    double yn_1 = 0, xn_1 = 0;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
}