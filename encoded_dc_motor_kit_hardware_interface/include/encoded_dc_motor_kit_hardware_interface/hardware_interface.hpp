#ifndef HWINTERFACE_HPP
#define HWINTERFACE_HPP

#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <libserial/SerialPort.h>
#include "sensor_msgs/msg/joint_state.hpp"
#include "ros2_motor_controller_msgs/msg/velocity_feedback.hpp"

#include <vector>
#include <string>
#include <sstream>

namespace encoded_dc_motor_kit_hardware_interface
{
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    class EncodedDcMotorKitHardwareInterface : public hardware_interface::SystemInterface
    {
        public:
            EncodedDcMotorKitHardwareInterface();

            // Destructor
            virtual ~EncodedDcMotorKitHardwareInterface();

           
            // Implementing rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface
            virtual CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
            virtual CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

            // Implementing hardware_interface::SystemInterface
            virtual CallbackReturn on_init(const hardware_interface::HardwareInfo &hardware_info) override;
            virtual std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
            virtual std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
            virtual hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
            virtual hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
            bool waitForData(LibSerial::SerialPort& serial_port, int timeout_ms);

            std::vector<double> parseDoubles(const std::string &data);

            std::vector<double> velocity_state;
            std::vector<double> velocity_command;
            std::vector<double> acceleration_state;
            std::vector<double> acceleration_command;
            std::vector<double> position_state;
            std::vector<double> position_command;
            std::vector<double> effort_state;
            std::vector<double> effort_command;
            double position = 0.0;
            double previous_position = 0.0;
            rclcpp::Publisher<ros2_motor_controller_msgs::msg::VelocityFeedback>::SharedPtr velocity_publisher_;
            rclcpp::TimerBase::SharedPtr timer_;
            rclcpp::Node::SharedPtr node_;
        

        private:
            LibSerial::SerialPort serial_port_;
            std::string port_;

    };
}

#endif