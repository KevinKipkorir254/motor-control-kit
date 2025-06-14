#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <cstdlib>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <iomanip>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

// I am writing some random stuff so I can get a new space for git
using namespace std::chrono_literals;
using std::placeholders::_1;

class velocity_and_frequency_terminal_display : public rclcpp::Node
{
public:
    velocity_and_frequency_terminal_display()
        : Node("velocity_and_frequency_terminal_display"), msg_count_(0), last_msg_count_(0), frequency_(0.0)
    {
        // Clear screen and hide cursor moved this here to allow seeing the motor-kit command
        std::cout << "\033[2J\033[H\033[?25l" << std::flush;

        // Text to display using figlet
        std::string text = "MOTOR-KIT";
        std::string command = "figlet -w $(tput cols) -c \"" + text + "\" | lolcat";
        std::system(command.c_str());
        
        text = "DASHBOARD";
        command = "figlet -f mini -w $(tput cols) -c \"" + text + "\" | lolcat";
        std::system(command.c_str());

        joint_state_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/filtered_velocity", 10, 
            std::bind(&velocity_and_frequency_terminal_display::update_velocity_display_value, this, std::placeholders::_1));

        // Timer for updating display
        display_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&velocity_and_frequency_terminal_display::update_display, this));

        // Timer for calculating frequency
        frequency_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&velocity_and_frequency_terminal_display::calculate_frequency, this));
        
        // Initial display
        draw_dashboard();
    }

    ~velocity_and_frequency_terminal_display()
    {
        // Show cursor
        std::cout << "\033[?25h" << std::flush;
    }

private:
    void update_velocity_display_value(const sensor_msgs::msg::JointState &msg)
    {
        // Increment message count for frequency calculation
        msg_count_++;
        
        // Get shaft position and velocity
        //auto shaft_it = std::find(msg.name.begin(), msg.name.end(), "");
        auto shaft_it = true; //TODO RUN CORRECT THIS TEMPORARY FIX beacuse I now it is present
        if (shaft_it == true)
        {
            //int shaft_index = std::distance(msg.name.begin(), shaft_it);
            //shaft_position_ = msg.position[0]; THIS IS WHAT CAUSED THE SEGMENTATION FAULT
            shaft_velocity_ = msg.velocity[0]; 
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Shaft not found");
            return;
        }
    }

    void calculate_frequency()
    {
        frequency_ = msg_count_ - last_msg_count_;
        last_msg_count_ = msg_count_;
    }

    void update_display()
    {
        draw_dashboard();
    }

    void draw_dashboard()
    {
        // Move cursor to dashboard position (after the figlet text)
        std::cout << "\033[10;1H";
        
        // Draw the semicircular speed gauge
        draw_speed_gauge();
        
        // Draw frequency display
        draw_frequency_display();
        
        std::cout << std::flush;
    }

    void draw_speed_gauge()
    {
        const int gauge_width = 60;
        const int gauge_height = 15;
        const double min_speed = -10.0;
        const double max_speed = 10.0;
        
        // Clamp velocity to display range
        double display_velocity = shaft_velocity_;
        if (display_velocity > max_speed) display_velocity = max_speed;
        if (display_velocity < min_speed) display_velocity = min_speed;
        
        // Calculate needle position
        double normalized_speed = (display_velocity - min_speed) / (max_speed - min_speed);
        if (normalized_speed < 0) normalized_speed = 0;
        if (normalized_speed > 1) normalized_speed = 1;
        
        // Semicircle angle from 0 to π (180 degrees)
        double needle_angle = M_PI * (1.0 - normalized_speed);  // Reverse for left-to-right
        
        std::cout << "\n";
        std::cout << "┌─────────────────────── SPEED GAUGE ───────────────────────┐\n";
        
        // Draw the gauge
        /*
        for (int row = 0; row < gauge_height; row++) {
            std::cout << "│";
            
            for (int col = 0; col < gauge_width; col++) {
                double x = (col - gauge_width/2.0) / (gauge_width/2.0);
                double y = (gauge_height - row - 1.0) / gauge_height;
                
                double distance = sqrt(x*x + y*y);
                double angle = atan2(y, x);
                
                // Normalize angle to 0-π range
                if (angle < 0) angle += 2*M_PI;
                
                char display_char = ' ';
                
                // Draw gauge arc
                if (distance > 0.8 && distance < 1.0 && angle >= 0 && angle <= M_PI) {
                    display_char = '●';
                }
                // Draw gauge ticks
                else if (distance > 0.7 && distance < 0.8 && angle >= 0 && angle <= M_PI) {
                    // Major ticks at -10, -5, 0, 5, 10
                    double tick_angles[] = {M_PI, 3*M_PI/4, M_PI/2, M_PI/4, 0};
                    for (double tick_angle : tick_angles) {
                        if (abs(angle - tick_angle) < 0.1) {
                            display_char = '|';
                            break;
                        }
                    }
                }
                // Draw needle
                else if (distance > 0.1 && distance < 0.7) {
                    double needle_diff = abs(angle - needle_angle);
                    if (needle_diff < 0.05) {
                        display_char = '█';
                    }
                }
                // Draw center
                else if (distance < 0.1) {
                    display_char = '●';
                }
                
                std::cout << display_char;
            }
            std::cout << "│\n";
        }
        */
        
        // Draw scale labels
        std::cout << "│";
        std::cout << std::setw(6) << "-10" << std::setw(10) << "-5" << std::setw(12) << "0" << std::setw(12) << "5" << std::setw(16) << "10";
        std::cout << "│\n";
        
        // Display current velocity value
        std::cout << "│" << std::setw(25) << "Shaft Velocity: ";
        std::cout << std::fixed << std::setprecision(2) << shaft_velocity_;
        std::cout << std::setw(22) << "│\n";
        
        // Display shaft position
        std::cout << "│" << std::setw(25) << "Shaft Position: ";
        std::cout << std::fixed << std::setprecision(2) << shaft_position_;
        std::cout << std::setw(22) << "│\n";
        
        std::cout << "└────────────────────────────────────────────────────────────┘\n";
    }

    void draw_frequency_display()
    {
        std::cout << "\n";
        std::cout << "┌──────────────── /filtered_velocity FREQUENCY ─────────────────┐\n";
        std::cout << "│" << std::setw(30) << "Topic Frequency: ";
        std::cout << std::fixed << std::setprecision(1) << frequency_ << " Hz";
        std::cout << std::setw(23) << "│\n";
        std::cout << "│" << std::setw(30) << "Total Messages: ";
        std::cout << msg_count_;
        std::cout << std::setw(28) << "│\n";
        std::cout << "└────────────────────────────────────────────────────────────────┘\n";
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
    rclcpp::TimerBase::SharedPtr display_timer_;
    rclcpp::TimerBase::SharedPtr frequency_timer_;
    
    volatile double shaft_position_ = 0.0;
    volatile double shaft_velocity_ = 0.0;
    
    // Frequency calculation variables
    int msg_count_;
    int last_msg_count_;
    double frequency_;
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