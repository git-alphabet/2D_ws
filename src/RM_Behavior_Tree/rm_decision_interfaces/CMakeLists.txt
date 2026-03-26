cmake_minimum_required(VERSION 3.10)
project(rm_decision_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  # "msg/RMUC.msg"  # 已拆分为 8 个独立消息，不再编译
  # ── RMUC 拆分后的独立消息 ──
  "msg/RMUC/RMUCGameStatus.msg"
  "msg/RMUC/RMUCRobotStatus.msg"
  "msg/RMUC/RMUCRFIDStatus.msg"
  "msg/RMUC/RMUCRobotPosition.msg"
  "msg/RMUC/RMUCEnemyTracks.msg"
  "msg/RMUC/RMUCSentryCmd.msg"
  "msg/RMUC/RMUCRobotControl.msg"
  "msg/RMUC/RMUCNavControlCmd.msg"
  DEPENDENCIES
    std_msgs
    geometry_msgs
    builtin_interfaces
)

ament_package()