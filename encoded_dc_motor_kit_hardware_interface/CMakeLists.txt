cmake_minimum_required(VERSION 3.8)
project(encoded_dc_motor_kit_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(ros2_motor_controller_msgs REQUIRED)


pkg_check_modules(SERIAL libserial)


# Add the plugin XML to the install
install(FILES
    encoded_dc_motor_kit_hardware_interface.xml
    DESTINATION share/${PROJECT_NAME}/
)


add_library(
  ${PROJECT_NAME} SHARED
  src/hardware_interface.cpp
  src/stop_and_wait.cpp
)

target_include_directories(
  ${PROJECT_NAME} PRIVATE
  include
  "${SERIAL_INCLUDE_DIRS}"
)

target_link_libraries(
  ${PROJECT_NAME}
  "${SERIAL_LDFLAGS}"
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp_lifecycle
  rclcpp
  pluginlib
  hardware_interface
  ros2_motor_controller_msgs
)

pluginlib_export_plugin_description_file(hardware_interface encoded_dc_motor_kit_hardware_interface.xml)

ament_python_install_package(${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(
  DIRECTORY include
  DESTINATION include
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# install(PROGRAMS
#   ${PROJECT_NAME}/slider_control.py
#   DESTINATION lib/${PROJECT_NAME}
# )

ament_export_include_directories(
  include()
)

ament_export_libraries(
  ${PROJECT_NAME}
)

ament_export_dependencies(
  rclcpp_lifecycle
  rclcpp
  pluginlib
  hardware_interface  
  ros2_motor_controller_msgs
)



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

ament_package()
