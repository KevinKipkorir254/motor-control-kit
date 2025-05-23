cmake_minimum_required(VERSION 3.8)
project(encoded_dc_motor_kit_state_space_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
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

add_executable(pole_placement_contoller src/encoded_dc_motor_kit_pole_placement_controller.cpp)
ament_target_dependencies(pole_placement_contoller rclcpp std_msgs sensor_msgs geometry_msgs)

#encoded_dc_motor_kit_LQR_controller.cpp

add_executable(LQR_contoller src/encoded_dc_motor_kit_LQR_controller.cpp)
ament_target_dependencies(LQR_contoller rclcpp std_msgs sensor_msgs geometry_msgs)

install(TARGETS
    pole_placement_contoller
    LQR_contoller
  DESTINATION lib/${PROJECT_NAME})

ament_package()
