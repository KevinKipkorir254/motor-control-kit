#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <cstdlib>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class velocity_and_frequency_terminal_display : public rclcpp::Node //velocity_and_frequency_terminal_display
{
public:
    velocity_and_frequency_terminal_display()
        : Node("velocity_and_frequency_terminal_display")
    {
        // Text to display using figlet
        std::string text = "MOTOR-KIT";
        std::string command = "figlet -w $(tput cols) -c \"" + text + "\" | lolcat";
        std::system(command.c_str());

        
        joint_state_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>("/filtered_velocity", 10, std::bind(&velocity_and_frequency_terminal_display::update_velocity_display_value, this, std::placeholders::_1));

    }
    
private:

    void update_velocity_display_value (const sensor_msgs::msg::JointState &msg)
    {
        // get shaft position
        auto shaft_it = std::find(msg.name.begin(), msg.name.end(), "shaft_joint");
        if (shaft_it != msg.name.end())
        {
            int shaft_index = std::distance(msg.name.begin(), shaft_it);
            shaft_position_ = msg.position[shaft_index];
            shaft_velocity_ = msg.velocity[shaft_index];

        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Shaft not found");
            return;
        }

    }


    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
    volatile double shaft_position_ = 0.0;
    volatile double shaft_velocity_ = 0.0;
    
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<velocity_and_frequency_terminal_display>();
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}