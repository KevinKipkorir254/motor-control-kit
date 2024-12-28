cmake_minimum_required(VERSION 3.8)
project(encoded_dc_motor_kit_kalman_filters)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# Find Eigen3
find_package(Eigen3 REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


# Include directories
include_directories(${EIGEN3_INCLUDE_DIRS})

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

# Include directories
add_executable(kalman_filter_with_arx_model src/shaft_velocity_filter.cpp)
ament_target_dependencies(kalman_filter_with_arx_model rclcpp std_msgs sensor_msgs Eigen3)


add_executable(kalman_filter_with_bj_model src/shaft_velocity_filter_bj.cpp)
ament_target_dependencies(kalman_filter_with_bj_model rclcpp std_msgs sensor_msgs Eigen3)

add_executable(kalman_filter_with_arxmax_model src/shaft_velocity_filter_armax.cpp)
ament_target_dependencies(kalman_filter_with_arxmax_model rclcpp std_msgs sensor_msgs Eigen3)


# Include directories
target_include_directories(kalman_filter_with_arx_model PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Install the target
install(TARGETS
  kalman_filter_with_arx_model
  kalman_filter_with_bj_model
  kalman_filter_with_arxmax_model
  DESTINATION lib/${PROJECT_NAME})

ament_package()
