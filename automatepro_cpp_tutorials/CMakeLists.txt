cmake_minimum_required(VERSION 3.22)
project(automatepro_cpp_tutorials)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(automatepro_interfaces REQUIRED)

# Add executables and link dependencies
add_executable(io_controller_diagnostic_node src/diagnostics/io_controller.cpp)
ament_target_dependencies(io_controller_diagnostic_node rclcpp automatepro_interfaces)

add_executable(analog_in_node src/io/analog_in.cpp)
ament_target_dependencies(analog_in_node rclcpp automatepro_interfaces)

add_executable(digital_drive_out_node src/io/digital_drive_out.cpp)
ament_target_dependencies(digital_drive_out_node rclcpp automatepro_interfaces)

add_executable(digital_in_node src/io/digital_in.cpp)
ament_target_dependencies(digital_in_node rclcpp automatepro_interfaces)

add_executable(digital_out_node src/io/digital_out.cpp)
ament_target_dependencies(digital_out_node rclcpp automatepro_interfaces)

add_executable(warning_system_out_node src/io/warning_system_out.cpp)
ament_target_dependencies(warning_system_out_node rclcpp automatepro_interfaces)

add_executable(gnss_heading_node src/sensors/gnss_heading.cpp)
ament_target_dependencies(gnss_heading_node rclcpp sensor_msgs)

add_executable(gnss_position_node src/sensors/gnss_position.cpp)
ament_target_dependencies(gnss_position_node rclcpp sensor_msgs)

add_executable(imu_node src/sensors/imu.cpp)
ament_target_dependencies(imu_node rclcpp sensor_msgs)

install(TARGETS
  io_controller_diagnostic_node
  analog_in_node
  digital_drive_out_node
  digital_in_node
  digital_out_node
  warning_system_out_node
  gnss_heading_node
  gnss_position_node
  imu_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_cppcheck)
  ament_lint_auto_find_test_dependencies()
  # ament_cppcheck skips cppcheck 2.x as too slow unless this variable is set.
  ament_cppcheck()
  set_tests_properties(cppcheck PROPERTIES ENVIRONMENT "AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1")
endif()

ament_package()
