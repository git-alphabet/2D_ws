cmake_minimum_required(VERSION 3.8)
project(ros2_bag_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/record_bag
  scripts/play_bag
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  topics
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
