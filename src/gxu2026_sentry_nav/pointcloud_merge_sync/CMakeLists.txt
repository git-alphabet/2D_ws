cmake_minimum_required(VERSION 3.8)
project(pointcloud_merge_sync)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

add_executable(pointcloud_merge_sync_node src/pointcloud_merge_sync_node.cpp)
ament_target_dependencies(pointcloud_merge_sync_node
  rclcpp sensor_msgs std_msgs message_filters pcl_conversions pcl_ros)

install(TARGETS
  pointcloud_merge_sync_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
