#include "encoded_dc_motor_kit_hardware_interface/hardware_interface.hpp"
#include "encoded_dc_motor_kit_hardware_interface/stop_and_wait.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

// ANSI escape codes for colors
#define GREEN_TEXT "\033[0;32m"
#define BLUE_TEXT "\033[0;34m"
#define RED_TEXT "\033[0;31m"
#define RESET_COLOR "\033[0m"

// sweet

namespace encoded_dc_motor_kit_hardware_interface
{
  EncodedDcMotorKitHardwareInterface::EncodedDcMotorKitHardwareInterface()
  {
    RCLCPP_INFO(rclcpp::get_logger("EncodedDcMotorKitHardwareInterface"), "In constructor...............");
  }

  EncodedDcMotorKitHardwareInterface::~EncodedDcMotorKitHardwareInterface()
  {
    if (serial_port_.IsOpen())
    {
      serial_port_.Close();
      RCLCPP_INFO(rclcpp::get_logger("hardware_interface_node"), "Serial Port closed.");
    }
  }

  CallbackReturn EncodedDcMotorKitHardwareInterface::on_init(const hardware_interface::HardwareInfo &hardware_info)
  {
    RCLCPP_INFO(rclcpp::get_logger("EncodedDcMotorKitHardwareInterface"), "In on init...............");

    // Initialize your hardware interface here
    CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
    if (result != CallbackReturn::SUCCESS)
    {
      return result;
    }

    // Print out all joints and set initial values to zero
    for (const auto &joint : info_.joints)
    {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("EncodedDcMotorKitHardwareInterface"),
                         "Joint name: " << joint.name);
    }

    // Reserve space for velocity command and state
    velocity_command.resize(info_.joints.size(), 0.0);
    velocity_state.resize(info_.joints.size(), 0.0);
    position_state.resize(info_.joints.size(), 0.0);
    position_command.resize(info_.joints.size(), 0.0);
    acceleration_state.resize(info_.joints.size(), 0.0);
    acceleration_command.resize(info_.joints.size(), 0.0);
    effort_command.resize(info_.joints.size(), 0.0);

    node_ = rclcpp::Node::make_shared("hardware_interface_node");
    velocity_publisher_ = node_->create_publisher<ros2_motor_controller_msgs::msg::VelocityFeedback>("velocity_feedback", 10);

    try
    {
      serial_port_.Open("/dev/ttyUSB0");
      serial_port_.SetBaudRate(LibSerial::BaudRate::BAUD_230400);
      serial_port_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
      serial_port_.SetParity(LibSerial::Parity::PARITY_NONE);
      serial_port_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
      serial_port_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);

      RCLCPP_INFO_STREAM(rclcpp::get_logger("hardware_interface_node"), "Serial Port initialized successfully.");
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(rclcpp::get_logger("hardware_interface_node"), "Serial port initialization failed: %s", e.what());
    }

    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> EncodedDcMotorKitHardwareInterface::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (size_t i = 0; i < info_.joints.size(); i++)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface{info_.joints[i].name, hardware_interface::HW_IF_ACCELERATION, &acceleration_state[i]});
      state_interfaces.emplace_back(hardware_interface::StateInterface{info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_state[i]});
      state_interfaces.emplace_back(hardware_interface::StateInterface{info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_state[i]});
    }

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> EncodedDcMotorKitHardwareInterface::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (size_t i = 0; i < info_.joints.size(); i++)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface{info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &effort_command[i]});
    }

    return command_interfaces;
  }

  CallbackReturn EncodedDcMotorKitHardwareInterface::on_activate(const rclcpp_lifecycle::State &previous_state)
  {
    RCLCPP_INFO(rclcpp::get_logger("EncodedDcMotorKitHardwareInterface"), "Starting robot hardware ...");

    velocity_state.assign(info_.joints.size(), 0.0);

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn EncodedDcMotorKitHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state)
  {

    return CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type EncodedDcMotorKitHardwareInterface::read(const rclcpp::Time &time,
                                                                           const rclcpp::Duration &period)
  {

    uint16_t data_output = 0;
    int16_t signed_data;
    int sweet = read_data_data(&data_output, &serial_port_);

    if (!(sweet == 1))
    {
      RCLCPP_INFO(rclcpp::get_logger("EncodedDcMotorKitHardwareInterface"), "Data read failed");
    }
    else
    {
      // Split data_output into two bytes
      uint8_t high_byte = (data_output >> 8) & 0xFF;
      uint8_t low_byte = data_output & 0xFF;

      // Combine them into a signed 16-bit integer
      signed_data = (int16_t)((high_byte << 8) | low_byte);
      RCLCPP_INFO(rclcpp::get_logger("EncodedDcMotorKitHardwareInterface"), GREEN_TEXT ": %d" RESET_COLOR, signed_data);
    }

    double position = (((double)signed_data) / 500)* 2 * M_PI; //convert to radians

    // calculate velocity
    double time_interval = period.seconds(); // get the time interval in seconds
    double velocity = (position - previous_position) / time_interval;
    previous_position = position;

    if ((velocity < 20.0) && (velocity > -20.0))
    {
      position_state[0] = position;
      velocity_state[0] = velocity;
    }
    else //state remains the same
    {
      position_state[0] = position_state[0];
      velocity_state[0] = velocity_state[0];
    }

    return hardware_interface::return_type::OK;
  }

  // Send the desired joint position, servoangles
  hardware_interface::return_type EncodedDcMotorKitHardwareInterface::write(const rclcpp::Time &time,
                                                                            const rclcpp::Duration &period)
  {
    int data_output_sting = (int)effort_command[0];
    // int data_output_sting = 0;

    //clamp PWM output value
    if (data_output_sting > 255)
    {
      data_output_sting = 255;
    }
    else if (data_output_sting < -255)
    {
      data_output_sting = -255;
    }


    // Split data_output into two bytes
    uint8_t high_byte = (data_output_sting >> 8) & 0xFF;
    uint8_t low_byte = data_output_sting & 0xFF;

    unsigned char data_output[2] = {(unsigned char)high_byte, (unsigned char)low_byte};
    int sweet = write_data_data(data_output, &serial_port_);

    if (!(sweet == 1))
    {
      RCLCPP_INFO(rclcpp::get_logger("EncodedDcMotorKitHardwareInterface"), "Data write failed");
    }
    else
    {
      RCLCPP_INFO(rclcpp::get_logger("EncodedDcMotorKitHardwareInterface"), BLUE_TEXT ": %d" RESET_COLOR, data_output_sting);
    }

    return hardware_interface::return_type::OK;
  }

} // namespace robotic_arm_hwinterface

PLUGINLIB_EXPORT_CLASS(encoded_dc_motor_kit_hardware_interface::EncodedDcMotorKitHardwareInterface, hardware_interface::SystemInterface)