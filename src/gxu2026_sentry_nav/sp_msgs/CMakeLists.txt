cmake_minimum_required(VERSION 3.8)
project(sp_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Float32Stamped.msg"
  "msg/RMUL.msg"
  # ── RMUC 拆分后的独立消息（全国赛） ──
  "msg/RMUC/RMUCGameStatus.msg"
  "msg/RMUC/RMUCRobotStatus.msg"
  "msg/RMUC/RMUCRFIDStatus.msg"
  "msg/RMUC/RMUCRobotPosition.msg"
  "msg/RMUC/RMUCEnemyTracks.msg"
  "msg/RMUC/RMUCSentryCmd.msg"
  "msg/RMUC/RMUCRobotControl.msg"
  "msg/RMUC/RMUCNavControlCmd.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
