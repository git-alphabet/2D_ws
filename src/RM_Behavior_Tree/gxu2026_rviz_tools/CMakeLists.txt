cmake_minimum_required(VERSION 3.8)
project(gxu2026_rviz_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(std_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  include/gxu2026_rviz_tools/toggle_calib_hint_tool.hpp
  include/gxu2026_rviz_tools/toggle_calib_markers_tool.hpp
  src/toggle_calib_hint_tool.cpp
  src/toggle_calib_markers_tool.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  Qt5::Core
  Qt5::Gui
  Qt5::Widgets
)

ament_target_dependencies(${PROJECT_NAME}
  ament_index_cpp
  pluginlib
  rclcpp
  rviz_common
  std_msgs
)

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES plugins_description.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY icons
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
