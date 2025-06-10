#include <chrono>
#include <functional>
#include <memory>
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

class velocity_publisher_server : public rclcpp::Node  //velocity_publisher_server
{
public:
    velocity_publisher_server()
        : Node("velocity_publisher_server")
    {
        // Text to display using figlet
        std::string text = "MOTOR-KIT";
        // Construct the figlet command with the -c option for centering
        std::string command = "figlet -w $(tput cols) -c \"" + text + "\" | lolcat"; //added lolcat
        std::system(command.c_str());
        text = "M";
        command = "figlet -w $(tput cols) -c \"" + text + "\" | lolcat"; //added lolcat
        std::system(command.c_str());
        text = "INCREASE";
        command = "figlet -f mini -w $(tput cols) -c \"" + text + "\" | lolcat"; //added lolcat
        // Execute the command
        std::system(command.c_str());
        text = "DECREASE";
        command = "figlet -f mini -w $(tput cols) -c \"" + text + "\" | lolcat"; //added lolcat
        std::system(command.c_str());
        text = "V";
        command = "figlet -w $(tput cols) -c \"" + text + "\" | lolcat"; //added lolcat
        // Execute the command
        std::system(command.c_str());

        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/velocity/commands", 10);
        
    }

private:
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<velocity_publisher_server>());
    rclcpp::shutdown();
    return 0;
}