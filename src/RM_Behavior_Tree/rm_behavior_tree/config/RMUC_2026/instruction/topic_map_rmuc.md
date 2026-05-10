# RMUC 2026 插件与 ROS2 接口映射表

本文档按当前会生成的 `sp_msgs/msg/RMUC/*.msg` 和 `config/RMUC_2026/*.xml` 整理。重点看三件事：

- 行为树真实订阅/发布了哪些 ROS 接口。
- 原始消息进入黑板后，哪些派生变量被战术子树消费。
- 当前姿态/导航/底盘控制的实际优先级。

## 订阅话题

| BT XML ID | 话题名 | 消息类型 | 黑板输出 | 当前用途 |
|---|---|---|---|---|
| `RmucSubGameStatus` | `game_status` | `sp_msgs/msg/RMUCGameStatus` | `{game_status}`, `{time.now_ms}` | 比赛阶段与剩余时间；`RmucIsGameTime` 和 `ParseSentryBlackboard` 消费 |
| `RmucSubRobotStatus` | `robot_status` | `sp_msgs/msg/RMUCRobotStatus` | `{robot_status}` | 血量、热量、允许发弹量、己方前哨站/基地血量、敌方前哨站状态、是否见敌 |
| `RmucSubRobotBuff` | `robot_buff` | `sp_msgs/msg/RMUCRobotBuff` | `{robot_buff}` | 易伤状态；`IsVulnerable` 消费 `vulnerability_pct` |
| `RmucSubRobotPosition` | `robot_position` | `sp_msgs/msg/RMUCRobotPosition` | `{pose.x}`, `{pose.y}`, `{is_at_nav_goal}` | 机器人位置与 Nav2 到达状态 |
| `SubRadarTracks` | `radar/enemy_tracks` | `sp_msgs/msg/RMUCEnemyTracks` | `{radar_tracks}` | 雷达敌人坐标；当前只用于基地威胁进入判断 |
| `IsNavTargetSupply` | `goal_pose` | `geometry_msgs/msg/PoseStamped` | 内部缓存 | 判断当前 Nav2 目标是否为补给区 |
| `IsAtGoal` | `global_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | 内部缓存 | 到点判断时做视线检查；无 costmap 时退化为距离判断 |

> `sentry_decision_status` 当前没有 RMUC 行为树订阅节点；`RMUCSentryDecisionStatus.msg` 已移入 `msg/pre_msg/` 归档，不再参与 `rosidl_generate_interfaces`。

## 发布话题

| BT XML ID | 话题名 | 消息类型 | 黑板输入 / 端口 | 当前用途 |
|---|---|---|---|---|
| `SentryCmdMux` | `sentry_cmd` | `sp_msgs/msg/RMUCSentryCmd` | `posture={cmd.final_posture}`, `confirm_respawn={cmd.confirm_respawn}` | 发布哨兵自主决策指令：姿态 + 确认复活 |
| `RmucRobotControl` | `robot_control` | `sp_msgs/msg/RMUCRobotControl` | `stop_gimbal_scan`, `chassis_spin` | 云台扫描开关与底盘小陀螺控制 |

### 当前未实例化的发布插件

| 插件 / 消息 | 状态 | 说明 |
|---|---|---|
| `RmucNavControlCmd` / `RMUCNavControlCmd` | 插件和 msg 仍存在，但当前 RMUC XML 未调用 | 停止导航已改为 `CancelNavGoal` 取消 Nav2 action；底盘停车/旋转由 `RmucRobotControl` 表达 |

`RMUCNavControlCmd.msg` 当前只剩 `cmd_type`，`emergency_stop` 已删除。

## RMUC 消息字段速查

下面把当前 RMUC 目录里的消息和已归档的旧消息单独列一遍，方便按“消息定义”查，而不是只看 topic 映射。

| 消息 | 话题 | 关键字段 | 当前 BT 使用情况 |
|---|---|---|---|
| `RMUCGameStatus` | `game_status` | `game_progress`, `stage_remain_time` | 已启用；`RmucSubGameStatus`、`RmucIsGameTime`、`ParseSentryBlackboard` 消费 |
| `RMUCRobotStatus` | `robot_status` | `current_hp`, `shooter_heat`, `ammo_allow`, `outpost_hp`, `base_hp`, `enemy_outpost_status`, `is_detect_enemy` | 已启用；当前是黑板解析主输入 |
| `RMUCRobotBuff` | `robot_buff` | `vulnerability_pct` | 已启用；`IsVulnerable` 直接消费 |
| `RMUCRobotPosition` | `robot_position` | `pose_x`, `pose_y`, `is_at_nav_goal` | 已启用；`RmucSubRobotPosition`、`IsAtGoal`、`IsNavTargetSupply` 相关逻辑消费 |
| `RMUCEnemyTracks` | `radar/enemy_tracks` | `enemy_x`, `enemy_y` | 已启用；`SubRadarTracks`、`ParseSentryBlackboard` 消费 |
| `RMUCSentryCmd` | `sentry_cmd` | `cmd_posture`, `cmd_confirm_respawn` | 已启用；`SentryCmdMux` 发布最终姿态指令 |
| `RMUCRobotControl` | `robot_control` | `stop_gimbal_scan`, `chassis_spin` | 已启用；`RmucRobotControl` 发布底盘/云台控制 |
| `RMUCNavControlCmd` | `nav_control_cmd` | `cmd_type` | 消息仍保留，但当前 RMUC XML 未实例化对应发布插件 |
| `RMUCSentryDecisionStatus` | `sentry_decision_status` | `current_posture`, `exchanged_ammo_total` | 已移入 `msg/pre_msg/` 归档；当前 BT 不订阅，也不生成接口 |

## 动作与导航接口

| BT XML ID | 接口名 | 类型 | 用途 |
|---|---|---|---|
| `SendGoal` | `goal_pose` | `geometry_msgs/msg/PoseStamped` 发布 | 发送 Nav2 目标点；目标点包括补给区、防御锚点、前哨站/高地/巡逻点 |
| `CancelNavGoal` | `navigate_to_pose/_action/cancel_goal` | `action_msgs/srv/CancelGoal` | 取消 Nav2 当前目标；普通见敌拦截时使用，同时清空 `SendGoal` 去重缓存 |

`SendGoal` 的 XML 端口中仍写 `action_name="navigate_to_pose"`，但当前实现实际发布 `goal_pose`，由外部 Nav2 桥接/订阅链路接收。

## 数据解析与派生变量

| 插件 | 类型 | 输入 | 关键输出 |
|---|---|---|---|
| `ParseSentryBlackboard` | `SyncAction` | `{game_status}`, `{robot_status}`, `{radar_tracks}`, `{pose.x/y}`, `cfg.*` | `game.*`, `hp.*`, `ammo.*`, `base.hp.*`, `outpost.*`, `enemy_outpost_*`, `state.*`, `combat.has_target`, `is_detect_enemy`, `threat.base` |

### 关键派生逻辑

- `state.is_dead`：由 `robot_status.current_hp <= 0` 得出。
- `is_detect_enemy`：来自 `robot_status.is_detect_enemy`，但机器人位于配置的语义忽略区域时会被压成 `false`。
- `outpost.alive`：由 `robot_status.outpost_hp > 0` 得出。
- `base.hp.cur`：由 `robot_status.base_hp` 得出。
- `enemy_outpost_destroyed`：敌方前哨站状态不是 `1/2` 时视为已摧毁。
- `threat.base`：基地威胁锁存。进入条件是雷达敌人靠近基地且基地掉血；解除条件是云台未见敌且基地不掉血持续 `base_threat_calm_timeout_ms`。
- `state.disengaged`：存活状态下连续 6 秒未发射且未掉血。

## 初始化与配置节点

| BT XML ID | 类型 | 说明 |
|---|---|---|
| `InitSentryConfig` | `SyncAction` | 从 ROS 参数/默认值写入 `cfg.*` |
| `LoadCalibrationCSV` | `SyncAction` | 从 `rmuc_calibration.csv` 覆盖地图坐标和巡逻点 |
| `InitCmdState` | `SyncAction` | 初始化 `{cmd.state}`、`{cmd.allow_ammo_target}`、补给阈值等命令状态 |

主树当前先执行 `InitOnce`，再执行 `PerceptionAndBlackboard`，保证同一 tick 内解析节点可以读到最新 `cfg.*`。

## 姿态管理

| BT XML ID | 类型 | 输入 | 输出 | 说明 |
|---|---|---|---|---|
| `SelectPosture` | `SyncAction` | `posture_value` | `{cmd.posture}`, `{cmd.allow_posture_send}` | 5 秒冷却防抖；冷却未到时保持旧姿态写回 `{cmd.posture}` |
| `PostureDegradationGuard` | `SyncAction` | `{cmd.posture}` | `{cmd.final_posture}` | 单姿态累计超过 180 秒后短暂强制切到另一姿态 |
| `SelectSentryCmdRate` | `SyncAction` | `{state.is_dead}` | `{cmd.sentry_cmd_hz}` | 存活 0.2Hz，死亡 1Hz |
| `SentryCmdMux` | ROS Pub | `{cmd.final_posture}`, `{cmd.confirm_respawn}` | `/sentry_cmd` | 实际发给电控/裁判系统的最终姿态 |

注意：`cmd.allow_posture_send` 当前没有接到 `SentryCmdMux`，所以它不是发布门控。冷却真正起效的是 `SelectPosture` 会把旧姿态继续写回 `{cmd.posture}`。

## 条件节点速查

| BT XML ID | 主要输入 | 判断 |
|---|---|---|
| `RmucIsGameTime` | `{game_status}`, `{robot_status}` | 比赛阶段和剩余时间窗口；允许无比赛状态时用 HP 回退判断 |
| `RmucIsDead` | `{robot_status}` | `current_hp <= 0` |
| `RmucIsHPBelow` | `{robot_status}`, `hp_threshold` | 当前 HP 低于阈值 |
| `IsAmmoBelow` | `{ammo.allow}`, `{cfg.ammo_low}` / `{supply.next_threshold}` | 允许发弹量不足 |
| `RmucIsDetectEnemy` | `{robot_status}`, `{is_detect_enemy}` | 优先使用解析后的 `is_detect_enemy` |
| `IsVulnerable` | `{robot_buff}` | `vulnerability_pct >= min_vulnerability_pct`，默认阈值 1 |
| `IsBaseThreatened` | `{threat.base}` | 基地威胁锁存是否为 true |
| `IsNavTargetSupply` | `goal_pose`, `{cfg.supply_zone_x/y}` | 当前导航目标是否落在补给区 |
| `IsAtGoal` | `{pose.x/y}`, `goal_x/y`, `arrive_radius` | 到点距离判断 + costmap 视线检查 + 滞回 |

## 快速浏览：话题到插件

| 话题 / 接口 | 方向 | 当前状态 | 关联插件 |
|---|---|---|---|
| `game_status` | SUB | 启用 | `RmucSubGameStatus` |
| `robot_status` | SUB | 启用 | `RmucSubRobotStatus` |
| `robot_buff` | SUB | 启用 | `RmucSubRobotBuff`, `IsVulnerable` |
| `robot_position` | SUB | 启用 | `RmucSubRobotPosition` |
| `radar/enemy_tracks` | SUB | 启用 | `SubRadarTracks`, `ParseSentryBlackboard` |
| `goal_pose` | PUB/SUB | 启用 | `SendGoal` 发布，`IsNavTargetSupply` 订阅 |
| `global_costmap/costmap` | SUB | 启用 | `IsAtGoal` |
| `sentry_cmd` | PUB | 启用 | `SentryCmdMux` |
| `robot_control` | PUB | 启用 | `RmucRobotControl` |
| `navigate_to_pose/_action/cancel_goal` | SERVICE | 启用 | `CancelNavGoal` |
| `sentry_decision_status` | SUB | 未被当前 BT 消费 | msg 已归档到 `pre_msg`，无订阅插件实例 |
| `nav_control_cmd` | PUB | 当前 BT 未实例化 | `RmucNavControlCmd` 插件保留 |

## 黑板命名空间

```text
cfg.*              -> 坐标、阈值、文件路径配置
time.*             -> 本地 tick 时间
game.*             -> 比赛剩余/已用时间
state.*            -> 生死、脱战等派生状态
hp.*               -> 哨兵当前血量
ammo.*             -> 允许发弹量
base.hp.*          -> 己方基地血量
outpost.*          -> 己方前哨站状态
threat.*           -> 威胁评估，当前主要是 threat.base
combat.*           -> 视觉战斗目标状态
pose.*             -> 自车位置
nav.*              -> 当前战略目标和最终导航目标
supply.*           -> 补弹失败计数与动态弹量阈值
cmd.*              -> 待发送指令、最终姿态、发送频率、内部命令状态
active_subtree     -> 当前接管分支，调试用
is_detect_enemy    -> 语义区过滤后的见敌标志
is_at_nav_goal     -> Nav2 当前目标是否已结束/到达
```

## 当前执行流程

```text
rmuc_2026 (ReactiveSequence)
|
+- InitOnce
|  +- InitSentryConfig
|  +- LoadCalibrationCSV
|  +- InitCmdState
|
+- PerceptionAndBlackboard
|  +- RmucSubGameStatus
|  +- RmucSubRobotStatus
|  +- RmucSubRobotBuff
|  +- RmucSubRobotPosition
|  +- SubRadarTracks
|  +- ParseSentryBlackboard
|
+- WhileDoElse(IsMatchStage: game_progress=4, remain 0..420s)
   |
   +- 比赛阶段 ReactiveSequence
   |  |
   |  +- CommandHub
   |  |  +- DecideRespawnCmd
   |  |  +- PostureDegradationGuard: cmd.posture -> cmd.final_posture
   |  |  +- SelectSentryCmdRate
   |  |  +- RateController(cmd.sentry_cmd_hz) -> SentryCmdMux -> sentry_cmd
   |  |
   |  +- GlobalNavigationAndChassisControl
   |  |  +- ChassisControlLayer -> robot_control
   |  |  +- UpdateHealRecoveryLatch
   |  |  +- SurvivalGuard
   |  |  |  +- 正常存活且不需回血 -> 放行
   |  |  |  +- 死亡/回血恢复锁存 -> LowHPRetreat 接管
   |  |  +- EnemyDetectionGuard
   |  |     +- 低弹 -> 放行给 SustainAndEconomy
   |  |     +- 基地威胁 -> 放行给 BaseDefense
   |  |     +- 易伤 -> SelectPosture(2) + KeepRunning
   |  |     +- 未见敌 -> 放行
   |  |     +- 当前目标是补给区 -> 放行
   |  |     +- 普通见敌 -> SelectPosture(1) + CancelNavGoal + robot_control(spin)
   |  |
   |  +- 战术 ReactiveFallback
   |     +- SustainAndEconomy: AmmoPlan 活跃时 SelectPosture(2)
   |     +- BaseDefense: 受威胁时去防御锚点；到点见敌攻姿态，否则移动姿态
   |     +- ObjectivePlanner: 选择前哨站/高地/巡逻目标；到点防御，路上移动
   |     +- DefaultPosture: 到当前 nav.goal 防御，否则移动
   |
   +- 非比赛阶段
      +- RmucRobotControl(stop_gimbal_scan=true, spin=false)
      +- SentryCmdMux(posture=3, confirm_respawn=0)
```

## 姿态优先级

当前不是单独一张“评分表”，而是由树的执行顺序决定：

1. `SurvivalGuard`：死亡/低血回血恢复锁存最高，进入 `LowHPRetreat` 后阻塞后续战术。
2. `SustainAndEconomy`：低弹补给高于基地威胁和普通见敌；补弹流程活跃时选择防御姿态。
3. `BaseDefense`：基地威胁高于普通见敌；基地威胁期间普通见敌拦截不会取消回防目标。
4. `VulnerableDefense`：易伤高于普通见敌，低于基地威胁；`vulnerability_pct` 未清零时保持防御姿态。
5. `EnemyHold`：普通见敌时攻击姿态、取消导航、原地旋转。
6. `ObjectivePlanner`：无高优先级接管时规划/巡逻；到点防御，路上移动。
7. `DefaultPosture`：兜底姿态；到当前目标防御，否则移动。
8. `PostureDegradationGuard`：最后对 `{cmd.posture}` 做累计时间降级，输出 `{cmd.final_posture}`。

## 外部数据源

| 组件 | 类型 | 发布/消费 | 说明 |
|---|---|---|---|
| `robot_position_bridge.py` | ROS2 节点 | 发布 `robot_position` | 仿真用：从 TF 和 Nav2 状态合成 `RMUCRobotPosition` |
| `rmuc_test_publisher.py` | 测试脚本 | 发布 `game_status`, `robot_status`, `radar/enemy_tracks` 等 | 仿真裁判系统数据源；具体字段随测试脚本实现变化 |

## 已删除或未接入项

- `RMUCNavControlCmd.emergency_stop` 已删除。
- `RMUCTeamHP` / `team_hp` 话题已废弃删除；己方前哨站和基地血量并入 `RMUCRobotStatus`。
- 当前 RMUC XML 不再调用 `RmucNavControlCmd`；导航停止由 `CancelNavGoal` 完成。
- `RMUCSentryDecisionStatus` 已归档到 `msg/pre_msg/`，当前 BT 不订阅，不作为姿态反馈闭环。
- `RMUCEnemyTracks.enemy_x/enemy_y` 仍为多目标格式；当前消费者只在基地威胁判断里遍历 `enemy_x/enemy_y`。
