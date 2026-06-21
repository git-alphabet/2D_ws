cmake_minimum_required(VERSION 3.8)
project(waypoint_follower_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(waypoint_follower_node
  src/waypoint_follower_node.cpp
)

target_include_directories(waypoint_follower_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(waypoint_follower_node
  rclcpp
  rclcpp_action
  geometry_msgs
  visualization_msgs
  nav2_msgs
  action_msgs
  std_srvs
)

install(TARGETS waypoint_follower_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
