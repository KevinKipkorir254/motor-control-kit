cmake_minimum_required(VERSION 3.8)
project(encoded_dc_motor_kit_compensator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(lead_compensator src/encoded_dc_motor_kit_lead_compensator.cpp)
ament_target_dependencies(lead_compensator rclcpp std_msgs sensor_msgs)

add_executable(lag_compensator src/encoded_dc_motor_kit_lag_compensator.cpp)
ament_target_dependencies(lag_compensator rclcpp std_msgs sensor_msgs)

add_executable(lag_lead_compensator src/encoded_dc_motor_kit_lead_lag_compensator.cpp)
ament_target_dependencies(lag_lead_compensator rclcpp std_msgs sensor_msgs)

add_executable(lead_compensator_position src/encoded_dc_motor_kit_lead_compensator_position.cpp)
ament_target_dependencies(lead_compensator_position rclcpp std_msgs sensor_msgs)

add_executable(lag_compensator_position src/encoded_dc_motor_kit_lag_compensator_position.cpp)
ament_target_dependencies(lag_compensator_position rclcpp std_msgs sensor_msgs)

add_executable(lag_lead_compensator_position src/encoded_dc_motor_kit_lead_lag_compensator_position.cpp)
ament_target_dependencies(lag_lead_compensator_position rclcpp std_msgs sensor_msgs)

install(TARGETS
  lead_compensator
  lag_compensator
  lag_lead_compensator
  lead_compensator_position
  lag_compensator_position
  lag_lead_compensator_position
  DESTINATION lib/${PROJECT_NAME})

ament_package()
