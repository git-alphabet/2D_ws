cmake_minimum_required(VERSION 3.8)
project(scan_additive_adapter)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(scan_additive_adapter_node src/scan_additive_adapter_node.cpp)
ament_target_dependencies(scan_additive_adapter_node rclcpp sensor_msgs)

install(TARGETS
  scan_additive_adapter_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
