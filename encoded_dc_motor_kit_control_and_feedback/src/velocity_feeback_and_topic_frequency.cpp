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
#include <sys/ioctl.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

// I am writing some random stuff so I can get a new space for git
using namespace std::chrono_literals;
using std::placeholders::_1;
struct winsize w;

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
    // Get terminal dimensions
    ioctl(STDOUT_FILENO, TIOCGWINSZ, &w);
    int terminal_width = w.ws_col;
    
    // Calculate starting position to center the 62-character wide boxes
    int box_width = 62; // Width of your bordered boxes
    int start_col = (terminal_width - box_width) / 2 + 1;
    
    // Position cursor at row 10, centered column
    std::cout << "\033[10;" << start_col << "H";
    
    // Draw the semicircular speed gauge
    draw_speed_gauge();
    
    box_width = 64; // This box is slightly wider
    start_col = (terminal_width - box_width) / 2 + 1;
    
    // Position cursor for frequency display (adjust row as needed)
    std::cout << "\033[" << (10 + 5) << ";" << start_col << "H"; // 8 rows down from speed gauge
    
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
    
    // Don't print newline here since we're positioning manually
    std::cout << "─────────────────────── SPEED GAUGE ────────────────────────";
    
    // Move to next line at same column position
    ioctl(STDOUT_FILENO, TIOCGWINSZ, &w);
    int terminal_width = w.ws_col;
    int box_width = 62;
    int start_col = (terminal_width - box_width) / 2 + 1;
    
    // Draw scale labels
    //
    //
    //
    //
    
    // Display current velocity value
    std::cout << "\033[" << (11) << ";" << start_col << "H";
    std::cout << std::setw(25) << "Shaft Velocity: ";
    std::cout << std::fixed << std::setprecision(2) << shaft_velocity_;
    //std::cout << std::setw(34) << "│";
    
    // Display shaft position
    std::cout << "\033[" << (12) << ";" << start_col << "H";
    std::cout << std::setw(25) << "Shaft Position: ";
    std::cout << std::fixed << std::setprecision(2) << shaft_position_;
    //std::cout << std::setw(34) << "│";
    
    std::cout << "\033[" << (13) << ";" << start_col << "H";
    std::cout << "────────────────────────────────────────────────────────────";
}

void draw_frequency_display()
{
    // Get current cursor position info
    ioctl(STDOUT_FILENO, TIOCGWINSZ, &w);
    int terminal_width = w.ws_col;
    int box_width = 64; // This box is slightly wider
    int start_col = (terminal_width - box_width) / 2 + 1;
    
    std::cout << "────────────────── filtered_velocity FREQUENCY ─────────────────";
    
    std::cout << "\033[" << (16) << ";" << start_col << "H";
    std::cout << std::setw(30) << "Topic Frequency: ";
    std::cout << std::fixed << std::setprecision(1) << frequency_ << " Hz";
    //std::cout << std::setw(31) << "│";
    
    std::cout << "\033[" << (17) << ";" << start_col << "H";
    std::cout << std::setw(30) << "Total Messages: ";
    std::cout << msg_count_;
    //std::cout << std::setw(36) << "│";
    
    std::cout << "\033[" << (18) << ";" << start_col << "H";
    std::cout << "────────────────────────────────────────────────────────────────";
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
