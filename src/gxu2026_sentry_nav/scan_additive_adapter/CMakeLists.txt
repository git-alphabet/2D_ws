cmake_minimum_required(VERSION 3.8)
project(scan_additive_adapter)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(scan_additive_adapter_node src/scan_additive_adapter_node.cpp)
ament_target_dependencies(scan_additive_adapter_node rclcpp sensor_msgs std_msgs message_filters tf2_ros tf2_geometry_msgs)

install(TARGETS
  scan_additive_adapter_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
