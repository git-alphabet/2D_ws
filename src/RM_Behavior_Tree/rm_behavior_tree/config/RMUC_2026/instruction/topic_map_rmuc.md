# RMUC 2026 插件 ↔ ROS2 话题/服务/动作 映射表

## 订阅话题 (Subscribers)

| 插件名 (BT XML ID) | 话题名 | 消息类型 | 黑板输出 | 说明 |
|---|---|---|---|---|
| **RmucSubGameStatus** | `game_status` | `sp_msgs/msg/RMUCGameStatus` | `game_status`, `now_ms` | 比赛状态（阶段、剩余时间） |
| **RmucSubRobotStatus** | `robot_status` | `sp_msgs/msg/RMUCRobotStatus` | `robot_status` (shared_ptr) | 机器人状态（血量、热量、弹量、基地血量、死亡标志等） |
| **RmucSubRFIDStatus** | `rfid_status` | `sp_msgs/msg/RMUCRFIDStatus` | `rfid.status` | RFID 区域检测（补给区/基地增益/前哨站增益） |
| **RmucSubRobotPosition** | `robot_position` | `sp_msgs/msg/RMUCRobotPosition` | `pose_x`, `pose_y`, `pose_yaw`, `is_at_nav_goal` | 机器人位姿 + 导航到达状态 |
| **SubRadarTracks** | `radar/enemy_tracks` | `sp_msgs/msg/RMUCEnemyTracks` | `radar_tracks` | 雷达敌方位置数据 |
| **RmucSubSentryDecisionStatus** | `sentry_decision_status` | `sp_msgs/msg/RMUCSentryDecisionStatus` | `sentry_decision_status` | 哨兵决策状态（复活、姿态、弹丸兑换） |
| **RmucSubRobotBuff** | `robot_buff` | `sp_msgs/msg/RMUCRobotBuff` | `robot_buff` | 增益状态（回血速率、冷却、防御、易伤） |
| **RmucSubProjectileAllowance** | `projectile_allowance` | `sp_msgs/msg/RMUCProjectileAllowance` | `projectile_allowance` | 发弹配额 + 堡垒弹丸 |
| **RmucSubFieldStatus** | `field_status` | `sp_msgs/msg/RMUCFieldStatus` | `field_status` | 场地资源状态（高地占领、能量机关、增益区） |
| **RmucSubEnemyMark** | `enemy_mark` | `sp_msgs/msg/RMUCEnemyMark` | `enemy_mark` | 敌方易损标记状态 |
| **RmucSubTeamPositions** | `team_positions` | `sp_msgs/msg/RMUCTeamPositions` | `team_positions` | 友方机器人位置 |
| **RmucSubTeamHP** | `team_hp` | `sp_msgs/msg/RMUCTeamHP` | `team_hp` | 友方血量统计 |

## 发布话题 (Publishers)

| 插件名 (BT XML ID) | 话题名 | 消息类型 | 黑板输入 | 说明 |
|---|---|---|---|---|
| **SentryCmdMux** | `sentry_cmd` | `sp_msgs/msg/RMUCSentryCmd` | `posture`, `confirm_respawn`, `confirm_instant_respawn`, `allow_ammo_target`, `trigger_remote_ammo`, `trigger_remote_hp` | 汇总所有决策输出为统一哨兵指令（RateController 5Hz 限频） |
| **RmucRobotControl** | `robot_control` | `sp_msgs/msg/RMUCRobotControl` | `stop_gimbal_scan`, `chassis_spin` | 机器人控制标志（云台扫描、小陀螺） |
| **RmucNavControlCmd** | `nav_control_cmd` | `sp_msgs/msg/RMUCNavControlCmd` | `cmd_type`, `emergency_stop` | 导航控制指令（用于 LowHPRetreat / BaseDefense / EngageCombat） |

## 动作调用 (Action Clients)

| 插件名 (BT XML ID) | 动作名 | 动作类型 | 说明 |
|---|---|---|---|
| **SendGoal** | `navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | 发送导航目标点（共享插件，用于回家、去补给区、去目标点、防御锚点等） |

---

## 数据解析节点

| 插件名 (BT XML ID) | 类型 | 说明 |
|---|---|---|
| **ParseSentryBlackboard** | SyncAction | 解析原始裁判系统消息，派生 40+ 黑板变量：时间、血量、热量、弹量、基地血量、复活状态、增益值、场地资源、敌方易损标记、队伍血量、**基地威胁判定**（锁存机制：敌人 <5m + 基地掉血 → 触发，所有敌人 >12m → 解除） |

## 初始化节点

| 插件名 (BT XML ID) | 类型 | 说明 |
|---|---|---|
| **InitSentryConfig** | SyncAction | 从 ROS 参数加载所有静态配置（地图坐标、阈值参数）到 `cfg.*` 黑板 |
| **InitCmdState** | SyncAction | 初始化 `cmd.state`（idle）和 `cmd.allow_ammo_target`（0） |

## 决策节点

| 插件名 (BT XML ID) | 类型 | 输入 | 输出 | 说明 |
|---|---|---|---|---|
| **DecidePosture** | SyncAction | `hp_cur/max`, `heat_cur/high`, `has_target`, `base_threat`, `stage_elapsed_time`, `now_ms`, `current_posture`, buff 值, `ammo_allow` | `posture_out` (1=攻击, 2=防御, 3=移动) | 综合评分选择姿态；5s 切换冷却 + 3min 后强制降级策略 |
| **DecideEconomyCmd** | SyncAction | `hp_cur`, `ammo_allow/target/low`, `team_coins`, `can_remote_heal/ammo`, `is_disengaged`, `allow_ammo_max`, `ammo_increase_interval_ms` | `allow_ammo_target_out`, `trigger_remote_ammo/hp` | 远程补血/补弹决策 + 弹丸配额累加（带限速） |
| **DecideRespawnCmd** | SyncAction | `is_dead`, `team_coins`, `stage_remain_time`, `base_hp_cur/max`, `instant_respawn_cost` | `confirm_respawn`, `confirm_instant_respawn` | 复活决策（免费/即时） |

## 状态持续节点

| 插件名 (BT XML ID) | 类型 | 说明 |
|---|---|---|
| **HoldAndHeal** | StatefulAction | 在治疗区等待直到 `hp_cur >= hp_safe`（默认 280） |
| **HoldForSupplyAmmoTick** | StatefulAction | 在补给区等待直到 `ammo_allow >= ammo_target`（默认 300） |
| **HoldObjective** | StatefulAction | 占点等待 `hold_ms`（默认 12000ms）；基地威胁或发现敌人时提前中断 |

## 规划/导航节点

| 插件名 (BT XML ID) | 类型 | 说明 |
|---|---|---|
| **SelectObjective** | SyncAction | 根据 `outpost_alive` 选择中央高地/梯形高地 |
| **SelectNearestResupplyStation** | SyncAction | 根据距离选择最近的补给/回血点（补给区/基地增益/前哨站增益） |
| **ObjectivePatrol** | SyncAction | 在目标点和巡逻航点间循环（`patrol_enable` + 仅梯形高地模式生效） |

## 条件节点 (无 ROS 接口)

| 插件名 (BT XML ID) | 输入 | 判断逻辑 |
|---|---|---|
| **IsAmmoBelow** | `ammo_allow`, `ammo_low` (默认 80) | `ammo_allow < ammo_low` |
| **IsAtGoal** | `pose_x/y`, `goal_x/y`, `arrive_radius` (默认 0.35m) | 欧氏距离 < arrive_radius |
| **IsBaseThreatened** | `base_threat` | 消费 ParseSentryBlackboard 派生的基地威胁标志 |
| **IsCombatAllowed** | `ammo_allow`, `heat_cur`, `heat_high`, `hp_cur`, `hp_low` | 弹量>0 AND 热量<上限 AND 血量>最低 |
| **RmucIsDead** | `robot_status` (shared_ptr) | `current_hp <= 0` |
| **RmucIsDetectEnemy** | `robot_status` (shared_ptr) | `is_detect_enemy == true` |
| **RmucIsGameTime** | `game_status`, `robot_status`, `game_progress`, `lower/higher_remain_time` | 比赛阶段 + 时间窗口检查（带血量回退） |
| **RmucIsHPBelow** | `robot_status` (shared_ptr), `hp_threshold` | `current_hp < threshold` |
| **RmucIsZoneCardDetected** | `zone` (SUPPLY/BASE/OUTPOST/CENTRAL_HIGHLAND/TRAPEZOIDAL_HIGHLAND/ENEMY_FORTRESS), `rfid_status` | 匹配 RFID 区域类型（6 种区域） |

---

## 快速浏览：话题 → 插件映射

| 话题 / 服务 / 动作 | 方向 | 关联插件 |
|---|---|---|
| `game_status` | SUB | RmucSubGameStatus |
| `robot_status` | SUB | RmucSubRobotStatus |
| `rfid_status` | SUB | RmucSubRFIDStatus |
| `robot_position` | SUB | RmucSubRobotPosition |
| `radar/enemy_tracks` | SUB | SubRadarTracks |
| `sentry_decision_status` | SUB | RmucSubSentryDecisionStatus |
| `robot_buff` | SUB | RmucSubRobotBuff |
| `projectile_allowance` | SUB | RmucSubProjectileAllowance |
| `field_status` | SUB | RmucSubFieldStatus |
| `enemy_mark` | SUB | RmucSubEnemyMark |
| `team_positions` | SUB | RmucSubTeamPositions |
| `team_hp` | SUB | RmucSubTeamHP |
| `sentry_cmd` | PUB | SentryCmdMux |
| `robot_control` | PUB | RmucRobotControl |
| `nav_control_cmd` | PUB | RmucNavControlCmd |
| `navigate_to_pose` | ACT | SendGoal (多处使用) |

> **方向说明**：SUB = 订阅，PUB = 发布，ACT = 动作调用

---

## 黑板变量命名空间

```
cfg.*              → 配置 (坐标、阈值、间隔)
time.*             → 时间
game.*             → 比赛状态
state.*            → 机器人状态
hp.*               → 血量
heat.*             → 枪口热量
ammo.*             → 弹药
base.hp.*          → 基地血量
outpost.*          → 前哨站状态
economy.*          → 经济 (金币、远程兑换)
combat.*           → 战斗状态
threat.*           → 威胁评估
sentry.*           → 哨兵专用 (姿态、复活)
buff.*             → 增益
field.*            → 场地资源
enemy.*            → 敌方标记
team.*             → 队伍 (前哨站/基地血量)
pose.*             → 位姿
cmd.*              → 指令输出
nav.*              → 导航状态
```

---

## 执行流程概览

```
rmuc_2026 (主树)
├─ PerceptionAndBlackboard (持续感知 KeepRunning)
│  ├─ RmucSub* (12个订阅者)
│  └─ ParseSentryBlackboard (数据解析 + 威胁判定)
├─ InitOnce (一次性初始化)
│  ├─ InitSentryConfig
│  └─ InitCmdState
└─ WhileDoElse (主循环)
   ├─ RmucIsGameTime (比赛阶段门控)
   ├─ ReactiveSequence (比赛中)
   │  ├─ CommandHub (决策中心)
   │  │  ├─ DecidePosture
   │  │  ├─ DecideEconomyCmd
   │  │  ├─ DecideRespawnCmd
   │  │  └─ SentryCmdMux (5Hz)
   │  └─ ReactiveFallback (优先级战术)
   │     ├─ [1] LowHPRetreat (死亡/低血)
   │     ├─ [2] BaseDefense (基地受威胁)
   │     ├─ [3] EngageCombat (有目标且安全)
   │     ├─ [4] SustainAndEconomy (补给逻辑)
   │     └─ [5] ObjectivePlanner (目标占领)
   └─ ReactiveSequence (非比赛阶段)
      └─ 回家 + 空闲指令
```
