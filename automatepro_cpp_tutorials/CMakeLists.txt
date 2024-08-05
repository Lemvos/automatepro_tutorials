cmake_minimum_required(VERSION 3.8)
project(automatepro_cpp_tutorials)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ublox_msgs REQUIRED)
find_package(automatepro_interfaces REQUIRED)

# Include directories
include_directories(include)

# Add executables and link dependencies
add_executable(io_controller_node src/diagnostics/io_controller.cpp)
ament_target_dependencies(io_controller_node rclcpp  std_msgs  automatepro_interfaces)

add_executable(analog_in_node src/io/analog_in.cpp)
ament_target_dependencies(analog_in_node rclcpp  std_msgs  automatepro_interfaces)

add_executable(digital_drive_out_node src/io/digital_drive_out.cpp)
ament_target_dependencies(digital_drive_out_node rclcpp std_srvs std_msgs  automatepro_interfaces)

add_executable(digital_in_node src/io/digital_in.cpp)
ament_target_dependencies(digital_in_node rclcpp std_srvs std_msgs  automatepro_interfaces)

add_executable(digital_out_node src/io/digital_out.cpp)
ament_target_dependencies(digital_out_node rclcpp  std_msgs  automatepro_interfaces)

add_executable(warning_systems_node src/io/warning_systems.cpp)
ament_target_dependencies(warning_systems_node rclcpp  std_msgs  automatepro_interfaces)

add_executable(gnss_heading_node src/sensors/gnss_heading.cpp)
ament_target_dependencies(gnss_heading_node rclcpp std_msgs sensor_msgs geometry_msgs ublox_msgs automatepro_interfaces)

add_executable(gnss_position_node src/sensors/gnss_position.cpp)
ament_target_dependencies(gnss_position_node rclcpp  std_msgs sensor_msgs geometry_msgs ublox_msgs automatepro_interfaces)

add_executable(imu_node src/sensors/imu.cpp)
ament_target_dependencies(imu_node rclcpp std_msgs sensor_msgs geometry_msgs )

install(TARGETS
  io_controller_node
  analog_in_node
  digital_drive_out_node
  digital_in_node
  digital_out_node
  warning_systems_node
  gnss_heading_node
  gnss_position_node
  imu_node
  DESTINATION lib/${PROJECT_NAME})

#install(DIRECTORY include/
#  DESTINATION include/${PROJECT_NAME})
#
#install(DIRECTORY launch
#  DESTINATION share/${PROJECT_NAME})
#
#install(DIRECTORY config
#  DESTINATION share/${PROJECT_NAME})

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
