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

class velocity_publisher_server : public rclcpp::Node
{
public:
    velocity_publisher_server()
        : Node("velocity_publisher_server"), current_velocity_(0.0)
    {
        // Text to display using figlet
        std::string text = "MOTOR-KIT";
        std::string command = "figlet -w $(tput cols) -c \"" + text + "\" | lolcat";
        std::system(command.c_str());
        
        text = "M";
        command = "figlet -w $(tput cols) -c \"" + text + "\" | lolcat";
        std::system(command.c_str());
        
        text = "INCREASE";
        command = "figlet -f mini -w $(tput cols) -c \"" + text + "\" | lolcat";
        std::system(command.c_str());
        
        text = "DECREASE";
        command = "figlet -f mini -w $(tput cols) -c \"" + text + "\" | lolcat";
        std::system(command.c_str());
        
        text = "V";
        command = "figlet -w $(tput cols) -c \"" + text + "\" | lolcat";
        std::system(command.c_str());

        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/velocity/commands", 10);
        
        // Setup keyboard input
        setupKeyboard();
        
        // Create a timer to check for keyboard input
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&velocity_publisher_server::checkKeyboard, this));
        
        // Display instructions
        //std::cout << "\n=== KEYBOARD CONTROLS ===" << std::endl;
        //std::cout << "UP Arrow    : Increase velocity by 0.1" << std::endl;
        //std::cout << "DOWN Arrow  : Decrease velocity by 0.1" << std::endl;
        //std::cout << "ESC or 'q'  : Quit" << std::endl;
        //std::cout << "Current velocity: " << current_velocity_ << std::endl;
        //std::cout << "=========================" << std::endl;
    }
    
    ~velocity_publisher_server()
    {
        restoreKeyboard();
    }

private:
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    double current_velocity_;
    struct termios old_termios_;
    
    void setupKeyboard()
    {
        // Get current terminal attributes
        tcgetattr(STDIN_FILENO, &old_termios_);
        
        struct termios new_termios = old_termios_;
        
        // Disable canonical mode and echo
        new_termios.c_lflag &= ~(ICANON | ECHO);
        new_termios.c_cc[VMIN] = 0;  // Non-blocking read
        new_termios.c_cc[VTIME] = 0;
        
        // Set new terminal attributes
        tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);
        
        // Set stdin to non-blocking
        int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    }
    
    void restoreKeyboard()
    {
        // Restore original terminal attributes
        tcsetattr(STDIN_FILENO, TCSANOW, &old_termios_);
    }
    
    void checkKeyboard()
    {
        char ch;
        if (read(STDIN_FILENO, &ch, 1) > 0)
        {
            if (ch == 27) // ESC sequence
            {
                // Read the next two characters for arrow keys
                char seq[2];
                if (read(STDIN_FILENO, &seq[0], 1) > 0 && read(STDIN_FILENO, &seq[1], 1) > 0)
                {
                    if (seq[0] == '[')
                    {
                        switch (seq[1])
                        {
                            case 'A': // Up arrow
                                increaseVelocity();
                                break;
                            case 'B': // Down arrow
                                decreaseVelocity();
                                break;
                        }
                    }
                }
                else
                {
                    // Just ESC key pressed
                    RCLCPP_INFO(this->get_logger(), "ESC pressed - shutting down...");
                    rclcpp::shutdown();
                }
            }
            else if (ch == 'q' || ch == 'Q')
            {
                RCLCPP_INFO(this->get_logger(), "'q' pressed - shutting down...");
                rclcpp::shutdown();
            }
        }
    }
    
    void increaseVelocity()
    {
        current_velocity_ += 0.1;
        publishVelocity();
        std::cout << "\rCurrent velocity: " << std::fixed << std::setprecision(1) << current_velocity_ << " (INCREASED)" << std::flush;
    }
    
    void decreaseVelocity()
    {
        current_velocity_ -= 0.1;
        publishVelocity();
        std::cout << "\rCurrent velocity: " << std::fixed << std::setprecision(1) << current_velocity_ << " (DECREASED)" << std::flush;
    }
    
    void publishVelocity()
    {
        auto message = std_msgs::msg::Float64MultiArray();
        message.data.push_back(current_velocity_);
        
        publisher_->publish(message);
        
        RCLCPP_DEBUG(this->get_logger(), "Published velocity: %f", current_velocity_);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<velocity_publisher_server>();
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}