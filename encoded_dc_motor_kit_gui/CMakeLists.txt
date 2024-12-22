cmake_minimum_required(VERSION 3.8)
project(encoded_dc_motor_kit_gui)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(std_msgs REQUIRED)

# Install python executables
install(
  PROGRAMS
    ${PROJECT_NAME}/feedback_subscriber_node.py
    ${PROJECT_NAME}/PID_parameters_publisher.py
    ${PROJECT_NAME}/response_plotter_node.py
    ${PROJECT_NAME}/voltage_publisher_gui.py
    ${PROJECT_NAME}/velocity_publisher_gui.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
