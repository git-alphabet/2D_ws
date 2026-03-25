# RMUL 2026 插件 ↔ ROS2 话题/服务/动作 映射表

## 订阅话题 (Subscribers)

| 插件名 (BT XML ID) | 话题名 | 消息类型 | 黑板输出 | 说明 |
|---|---|---|---|---|
| **SubRobotStatus** | `robot_status` | `rm_decision_interfaces/msg/RMUL` | `robot_status` | 机器人状态（血量、枪口热量、检测敌人等） |
| **SubGameStatus** | `game_status` | `rm_decision_interfaces/msg/RMUL` | `game_status`, `now_ms` | 比赛状态（阶段、剩余时间） |
| **SubRobotPosition** | `robot_position` | `rm_decision_interfaces/msg/RMUL` | `pose.x`, `pose.y` | 机器人位置（map 坐标系） |
| **SubArmors** | `detector/armors` | `auto_aim_interfaces/msg/Armors` | `armors` | 自瞄检测到的装甲板 |
| **SubRFIDStatus** | `rfid_status` | `rm_decision_interfaces/msg/RMUL` | `rfid.status` | RFID 交互状态（补给区/控制区到达） |
| **SubAllRobotHP** | *(可配置)* | `rm_decision_interfaces/msg/AllRobotHP` | `robot_hp` | 全场机器人血量 |
| **SubDecisionNum** | *(可配置)* | `rm_decision_interfaces/msg/DecisionNum` | `decision_num` | 决策路由编号 |
| **ExecuteNav2Waypoints** | `waypoints` | `visualization_msgs/msg/MarkerArray` | — | 航点可视化（rViz 打点输入） |
| **CalibrateCenterAnchor** | `/calib/spawn_point`<br>`/calib/control_point` | `geometry_msgs/msg/PointStamped` | `calib.*` | rViz Publish Point 校准 |
| **DetectRespawnAndSetRecovery** | `robot_status` | `rm_decision_interfaces/msg/RMUL` | `state.was_dead`, `state.need_recovery`, `hp.cur` 等 | 检测复活并设置恢复标志 |
| **WaitAndHeal** | *(默认)* | `rm_decision_interfaces/msg/RMUL` | `state.heal_start_ms` | 补给区等待回血 |
| **SentryFollower** | `armor_position` | `geometry_msgs/msg/PoseStamped` | — | 跟踪敌方装甲板位姿 |

## 发布话题 (Publishers)

| 插件名 (BT XML ID) | 话题名 | 消息类型 | 说明 |
|---|---|---|---|
| **SendGoal** | `goal_pose` | `geometry_msgs/msg/PoseStamped` | 发送导航目标点 |
| **RobotControl** | *(默认 topic)* | `rm_decision_interfaces/msg/RMUL` | 控制云台扫描、小陀螺 (`stop_gimbal_scan`, `chassis_spin`) |
| **NavControlCmd** | *(默认 topic)* | `rm_decision_interfaces/msg/RMUL` | 导航控制命令 (`cmd_type`, `emergency_stop`) |
| **MoveAround** | `goal_pose` | `geometry_msgs/msg/PoseStamped` | 随机移动目标点 |
| **MicroSearchSupplyCard** | `goal_pose` | `geometry_msgs/msg/PoseStamped` | 微搜索补给卡目标点 |
| **CalibrateCenterAnchor** | `/calibrated_points` | `visualization_msgs/msg/MarkerArray` | 校准点可视化标记 |

## 服务调用 (Service Clients)

| 插件名 (BT XML ID) | 服务名 | 服务类型 | 说明 |
|---|---|---|---|
| **ClearCostmap** | `local_costmap/clear_entirely` | `nav2_msgs/srv/ClearEntireCostmap` | 清空局部代价地图（脱困恢复用） |
| **CancelNavGoal** | `navigate_to_pose/_action/cancel_goal` | `action_msgs/srv/CancelGoal` | 取消导航目标 |

## 动作调用 (Action Clients)

| 插件名 (BT XML ID) | 动作名 | 动作类型 | 说明 |
|---|---|---|---|
| **ExecuteNav2Waypoints** | `navigate_through_poses` | `nav2_msgs/action/NavigateThroughPoses` | 多航点导航 |

## TF 查询

| 插件名 (BT XML ID) | 源坐标系 → 目标坐标系 | 说明 |
|---|---|---|
| **GetCurrentLocation** | `map` ↔ `gimbal_yaw` | 获取当前位姿 |
| **CalibrateCenterAnchor** | `map` ↔ `base_footprint` | 校准锚点位置 |
| **MicroSearchSupplyCard** | `map` ↔ `base_footprint` | 补给卡微搜索 |
| **SubRobotPosition** | `map` ↔ `base_footprint` (备选) | 位置 TF 回退 |
| **SentryFollower** | TF 跟踪 | 装甲板位姿转换 |

## 纯黑板条件节点 (无 ROS 接口)

| 插件名 (BT XML ID) | 输入 | 判断逻辑 |
|---|---|---|
| **IsDetectEnemy** | `robot_status` (RMUL msg) | `is_detect_enemy == true` |
| **IsDead** | `robot_status` | `current_hp <= 0` |
| **IsHPAbove** | `robot_status`, `hp_threshold` | `current_hp >= hp_threshold` |
| **IsHPBelow** | `robot_status`, `hp_threshold` | `current_hp < hp_threshold` |
| **IsStatusOK** | `robot_status`, 阈值 | HP 和热量均在安全范围 |
| **IsGameTime** | `game_status` | 比赛阶段和剩余时间 |
| **IsRecoveryNeeded** | `state.need_recovery` | 布尔检查 |
| **IsWithinScope** | `pose.x/y`, `goal_x/y`, `arrive_radius` | 距离 < arrive_radius |
| **IsNavigationStuck** | `pose.x/y`, `goal_x/y` | 近目标处移动检测（5s+锁存） |
| **IsRobotStuck** | `pose.x/y` | 全局移动检测（8s+锁存） |
| **IsControlZoneDetected** | `rfid.status` | `rfid_control_arrived == true` |
| **IsSupplyCardDetected** | `rfid.status` | RFID 补给卡检测 |
| **IsAtNavGoal** | `rfid.status` | `is_at_nav_goal == true` |
| **NotArrived** | `rfid.status` | `is_at_nav_goal` 取反 |
| **IsFriendOK** | `robot_status` | 队友状态检查 |

## 纯逻辑节点 (无 ROS 接口)

| 插件名 | 说明 |
|---|---|
| **InitBlackboardConfig** | 读取 ROS 参数初始化配置 (`is_blue_team`, 阈值等) |
| **SetGoalFromCenterOffset** | 从校准锚点 + 偏移计算目标点 |
| **SetNavGoalFromConfig** | 从配置读取目标点 |
| **InitSearchTimerIfNeeded** | 初始化搜索计时器 |
| **ClearRecoveryFlag** | 清除恢复标志 |
| **KeepRunning** | 返回 RUNNING 保持子树活跃 |
| **RateController** | 限频装饰器 |
| **DecisionSwitch** | 根据 decision_num 路由子节点 |
| **PrintMessage** | 调试打印 |

---

## 快速浏览：话题 → 插件映射

| 话题 / 服务 / 动作 | 方向 | 关联插件 |
|---|---|---|
| `robot_status` | SUB | SubRobotStatus, DetectRespawnAndSetRecovery, WaitAndHeal |
| `game_status` | SUB | SubGameStatus |
| `robot_position` | SUB | SubRobotPosition |
| `detector/armors` | SUB | SubArmors |
| `rfid_status` | SUB | SubRFIDStatus |
| `waypoints` | SUB | ExecuteNav2Waypoints |
| `armor_position` | SUB | SentryFollower |
| `/calib/spawn_point` | SUB | CalibrateCenterAnchor (出生点校准) |
| `/calib/control_point` | SUB | CalibrateCenterAnchor (控制区校准) |
| `goal_pose` | PUB | SendGoal, MoveAround, MicroSearchSupplyCard, SentryFollower |
| `/calibrated_points` | PUB | CalibrateCenterAnchor |
| *(RobotControl 默认 topic)* | PUB | RobotControl (`stop_gimbal_scan`, `chassis_spin`) |
| *(NavControlCmd 默认 topic)* | PUB | NavControlCmd (`cmd_type`, `emergency_stop`) |
| `local_costmap/clear_entirely` | SRV | ClearCostmap |
| `navigate_to_pose/_action/cancel_goal` | SRV | CancelNavGoal |
| `navigate_through_poses` | ACT | ExecuteNav2Waypoints |

> **方向说明**：SUB = 订阅，PUB = 发布，SRV = 服务调用，ACT = 动作调用
