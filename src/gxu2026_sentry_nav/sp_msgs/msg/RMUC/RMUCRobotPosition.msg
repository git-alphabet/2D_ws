# =============================================================================
# RMUCRobotPosition.msg — 机器人位姿与导航到达判定 (0x0203 / 定位系统)
# =============================================================================
# 话题: /robot_position        频率: 50 Hz
# 来源: 定位系统 (LiDAR SLAM)
# 订阅: RmucSubRobotPosition → 写入黑板 {pose.x} / {pose.y} / {pose.yaw}
# 引用: SelectBestTarget, SelectObjective, SelectNearestDispelCard,
#       SelectNearestResupplyStation, SelectSafeRetreatGoal, IsAtGoal,
#       IsAtNavGoal, WaypointPatrol, MoveAround
# =============================================================================

std_msgs/Header header

float32 pose_x               # 位置 x (meter, map 坐标系)
float32 pose_y               # 位置 y (meter, map 坐标系)
float32 pose_yaw             # 航向角 (radian, map 坐标系)
