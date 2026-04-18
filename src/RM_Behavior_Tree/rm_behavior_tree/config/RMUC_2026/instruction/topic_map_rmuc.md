# RMUC 2026 插件 ↔ ROS2 话题/服务/动作 映射表

## 订阅话题 (Subscribers)

| 插件名 (BT XML ID) | 话题名 | 消息类型 | 黑板输出 | 说明 |
|---|---|---|---|---|
| **RmucSubGameStatus** | `game_status` | `sp_msgs/msg/RMUCGameStatus` | `{game_status}`, `{time.now_ms}` | 比赛状态（阶段、剩余时间） |
| **RmucSubRobotStatus** | `robot_status` | `sp_msgs/msg/RMUCRobotStatus` | `{robot_status}` (shared_ptr) | 机器人状态（血量、热量、弹量、基地血量、死亡标志等） |
| **RmucSubRFIDStatus** | `rfid_status` | `sp_msgs/msg/RMUCRFIDStatus` | `{rfid.status}` | RFID 区域检测（补给区/基地增益/前哨站增益） |
| **RmucSubRobotPosition** | `robot_position` | `sp_msgs/msg/RMUCRobotPosition` | `{pose.x}`, `{pose.y}`, `{pose.yaw}`, `{is_at_nav_goal}` | 机器人位姿 + Nav2 到达状态（仿真由 `robot_position_bridge.py` 发布） |
| **SubRadarTracks** | `radar/enemy_tracks` | `sp_msgs/msg/RMUCEnemyTracks` | `{radar_tracks}` | 雷达敌方位置数据 |
| **RmucSubSentryDecisionStatus** | `sentry_decision_status` | `sp_msgs/msg/RMUCSentryDecisionStatus` | `{sentry_decision_status}` | 裁判系统哨兵决策反馈（姿态确认、复活、弹丸兑换） |
| **RmucSubRobotBuff** | `robot_buff` | `sp_msgs/msg/RMUCRobotBuff` | `{robot_buff}` | 增益状态（回血速率、冷却、防御、易伤） |
| **RmucSubProjectileAllowance** | `projectile_allowance` | `sp_msgs/msg/RMUCProjectileAllowance` | `{projectile_allowance}` | 发弹配额 + 堡垒弹丸 |
| **RmucSubFieldStatus** | `field_status` | `sp_msgs/msg/RMUCFieldStatus` | `{field_status}` | 场地资源状态（高地占领、能量机关、增益区） |
| **RmucSubEnemyMark** | `enemy_mark` | `sp_msgs/msg/RMUCEnemyMark` | `{enemy_mark}` | 敌方易损标记状态 |
| **RmucSubTeamHP** | `team_hp` | `sp_msgs/msg/RMUCTeamHP` | `{team_hp}` | 友方血量统计 |
| **IsNavTargetSupply** *(内部订阅)* | `goal_pose` | `geometry_msgs/msg/PoseStamped` | — | 订阅 Nav2 当前导航目标，判断是否正在回补给区 |

> **注意**：`team_positions` 订阅已禁用（幽灵端口，黑板键从未被消费）

## 发布话题 (Publishers)

| 插件名 (BT XML ID) | 话题名 | 消息类型 | 黑板输入 | 说明 |
|---|---|---|---|---|
| **SentryCmdMux** | `sentry_cmd` | `sp_msgs/msg/RMUCSentryCmd` | `{cmd.final_posture}`, `{cmd.confirm_respawn}`, `{cmd.confirm_instant_respawn}`, `{cmd.allow_ammo_target}`, `{cmd.trig_remote_ammo}`, `{cmd.trig_remote_hp}` | 汇总所有决策输出为统一哨兵指令（RateController 5Hz 限频） |
| **RmucRobotControl** | `robot_control` | `sp_msgs/msg/RMUCRobotControl` | `stop_gimbal_scan`, `chassis_spin` | 底盘旋转 / 云台扫描控制（GlobalRobotControl 根据条件发布） |
| **RmucNavControlCmd** | `nav_control_cmd` | `sp_msgs/msg/RMUCNavControlCmd` | `cmd_type`, `emergency_stop` | 导航控制指令（停车 / 底盘旋转停止等） |

## 动作调用 (Action Clients)

| 插件名 (BT XML ID) | 动作名 | 动作类型 | 说明 |
|---|---|---|---|
| **SendGoal** | `navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | 发送导航目标点（共用插件，用于回家/补给/目标点/防御锚点） |
| **CancelNavGoal** | `navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | 取消当前导航目标（EnemyDetectionGuard 停车时使用） |

---

## 数据解析节点

| 插件名 (BT XML ID) | 类型 | 说明 |
|---|---|---|
| **ParseSentryBlackboard** | SyncAction | 解析原始裁判系统消息 → 派生 60+ 黑板变量：时间、血量、热量、弹量、基地血量、复活状态、增益值、场地资源、敌方易损标记、队伍血量、**基地威胁判定**（锁存：敌人 <5m + 基地掉血 → 触发，所有敌人 >12m → 解除） |

## 初始化节点

| 插件名 (BT XML ID) | 类型 | 说明 |
|---|---|---|
| **InitSentryConfig** | SyncAction | 从 ROS 参数加载所有静态配置（地图坐标、阈值参数）到 `cfg.*` 黑板 |
| **LoadCalibrationCSV** | SyncAction | 从 CSV 文件加载标定坐标，覆盖 YAML 默认值 |
| **InitCmdState** | SyncAction | 初始化 `{cmd.state}`（idle）和 `{cmd.allow_ammo_target}`（0） |

## 姿态管理节点

| 插件名 (BT XML ID) | 类型 | 说明 |
|---|---|---|
| **SelectPosture** | SyncAction | 带 5s 冷却防抖的姿态选择器。输入 `posture_value` (1=进攻 2=防御 3=移动)，输出 `posture_out` → `{cmd.posture}` + `is_allow_select_posture` → `{cmd.allow_posture_send}` (门控：冷却中=false，阻止下游发送)。多个实例独立状态。 |
| **PostureDegradationGuard** | SyncAction | 姿态时间累计监管：单姿态累计超过180s → 强制切换姿态并重置计时器。攻击(1)/移动(3)超时→强制防御(2)5s；防御(2)超时→强制移动(3)5s（特殊情况）。输入 `{cmd.posture}` + `forced_movement_s`(默认10) → 输出 `{cmd.final_posture}`。 |

## 经济决策节点

| 插件名 (BT XML ID) | 类型 | 输入 | 输出 | 说明 |
|---|---|---|---|---|
| **DecideEconomyCmd** | SyncAction | `hp_cur`, `ammo_allow/target/low`, `team_coins`, `can_remote_heal/ammo`, `is_disengaged` (脱战状态：无敌人威胁时为true，控制远程补弹触发), `allow_ammo_max`, `ammo_increase_interval_ms` | `{cmd.allow_ammo_target}`, `{cmd.trig_remote_ammo}`, `{cmd.trig_remote_hp}` | 远程补血/补弹决策 + 弹丸配额累加（带限速） |
| **DecideRespawnCmd** | SyncAction | `is_dead`, `team_coins`, `stage_remain_time`, `base_hp_cur/max`, `instant_respawn_cost` | `{cmd.confirm_respawn}`, `{cmd.confirm_instant_respawn}` | 复活决策（免费/即时） |

## 状态持续节点

| 插件名 (BT XML ID) | 类型 | 说明 |
|---|---|---|
| **HoldAndHeal** | StatefulAction | 在治疗区等待直到 `hp_cur >= hp_safe`（默认 280） |
| **HoldForSupplyAmmoTick** | StatefulAction | 在补给区等待直到 `ammo_allow >= ammo_target`（默认 300） |

## 规划/导航节点

| 插件名 (BT XML ID) | 类型 | 说明 |
|---|---|---|
| **SelectObjective** | SyncAction | 根据 `outpost_alive` 选择中央高地/梯形高地 |
| **SelectNearestResupplyStation** | SyncAction | 根据距离选择最近的补给站（补给区/基地增益/前哨站增益） |
| **ObjectivePatrol** | SyncAction | 在目标点和巡逻航点间循环（`patrol_enable` + `patrol_waypoints`） |

## 条件节点 (无 ROS 接口)

| 插件名 (BT XML ID) | 输入 | 判断逻辑 |
|---|---|---|
| **IsAmmoBelow** | `{ammo.allow}`, `{cfg.ammo_low}` | `ammo_allow < ammo_low` |
| **IsAtGoal** | `{pose.x/y}`, `{goal_x/y}`, `arrive_radius` | 欧氏距离 < arrive_radius（参数化：`{cfg.supply_zone_radius}` / `{cfg.patrol_arrive_radius}` / `{cfg.base_defense_arrive_radius}`） |
| **IsBaseThreatened** | `{threat.base}` | 消费 ParseSentryBlackboard 派生的基地威胁标志（锁存机制：敌人 <`enemy_near_base_radius` + 基地掉血 → 触发，所有敌人 >12m → 解除） |
| **IsNavTargetSupply** | `{cfg.supply_zone_x/y}`, `{cfg.supply_zone_radius}` + 内部订阅 `goal_pose` | 当前 Nav2 导航目标在补给区范围内 |
| **RmucIsDead** | `{robot_status}` | `current_hp <= 0` |
| **RmucIsDetectEnemy** | `{robot_status}` | `is_detect_enemy == true` |
| **RmucIsGameTime** | `{game_status}`, `{robot_status}`, `game_progress`, `lower/higher_remain_time` | 比赛阶段 + 时间窗口检查（带血量回退） |
| **RmucIsHPBelow** | `{robot_status}`, `hp_threshold` | `current_hp < threshold` |
| **ScriptCondition** | 黑板变量表达式 | 通过脚本代码求值（如 `is_at_nav_goal == true`） |

---

## 快速浏览：话题 → 插件映射

| 话题 / 动作 | 方向 | 关联插件 |
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
| `team_hp` | SUB | RmucSubTeamHP |
| `goal_pose` | SUB | IsNavTargetSupply (内部) |
| `sentry_cmd` | **PUB** | SentryCmdMux |
| `robot_control` | **PUB** | RmucRobotControl |
| `nav_control_cmd` | **PUB** | RmucNavControlCmd |
| `navigate_to_pose` | **ACT** | SendGoal / CancelNavGoal |

> **方向说明**：SUB = 订阅，PUB = 发布，ACT = 动作调用

---

## 黑板变量命名空间

```
cfg.*              → 配置 (坐标、阈值、间隔)    — InitSentryConfig 输出
time.*             → 时间                        — RmucSubGameStatus 输出
game.*             → 比赛状态                    — ParseSentryBlackboard 派生
state.*            → 机器人状态                  — ParseSentryBlackboard 派生
hp.*               → 血量                        — ParseSentryBlackboard 派生
heat.*             → 枪口热量                    — ParseSentryBlackboard 派生
ammo.*             → 弹药                        — ParseSentryBlackboard 派生
base.hp.*          → 基地血量                    — ParseSentryBlackboard 派生
outpost.*          → 前哨站状态                  — ParseSentryBlackboard 派生
economy.*          → 经济 (金币、远程兑换)       — ParseSentryBlackboard 派生
threat.*           → 威胁评估                    — ParseSentryBlackboard 派生 (锁存机制)
sentry.*           → 哨兵专用 (姿态反馈、复活)   — ParseSentryBlackboard 派生
buff.*             → 增益                        — ParseSentryBlackboard 派生
field.*            → 场地资源                    — ParseSentryBlackboard 派生
enemy.*            → 敌方标记                    — ParseSentryBlackboard 派生
team.*             → 队伍 HP                     — ParseSentryBlackboard 派生
pose.*             → 位姿                        — RmucSubRobotPosition 输出
cmd.*              → 指令输出                    — SelectPosture / Decide* / PostureDegradationGuard
nav.*              → 导航状态                    — SelectObjective / ObjectivePatrol
is_at_nav_goal     → Nav2 到达状态 (bool)        — RmucSubRobotPosition 输出
active_subtree     → 当前活跃子树名 (调试用)     — SetBlackboard
```

---

## 执行流程概览

```
rmuc_2026 (ReactiveSequence — 所有子节点每 tick 重新求值)
│
├─ PerceptionAndBlackboard (SubTree, 持续订阅 + 解析)
│  ├─ RmucSubGameStatus       → {game_status}, {time.now_ms}
│  ├─ RmucSubRobotStatus      → {robot_status}
│  ├─ RmucSubRFIDStatus       → {rfid.status}
│  ├─ RmucSubRobotPosition    → {pose.x/y/yaw}, {is_at_nav_goal}
│  ├─ SubRadarTracks           → {radar_tracks}
│  ├─ RmucSubSentryDecisionStatus → {sentry_decision_status}
│  ├─ RmucSubRobotBuff        → {robot_buff}
│  ├─ RmucSubProjectileAllowance → {projectile_allowance}
│  ├─ RmucSubFieldStatus      → {field_status}
│  ├─ RmucSubEnemyMark        → {enemy_mark}
│  ├─ RmucSubTeamHP           → {team_hp}
│  └─ ParseSentryBlackboard   → 60+ 派生状态变量
│
├─ InitOnce (SubTree, 幂等)
│  ├─ InitSentryConfig         → cfg.* (YAML → 黑板)
│  ├─ LoadCalibrationCSV       → 覆盖坐标
│  └─ InitCmdState             → {cmd.state}, {cmd.allow_ammo_target}
│
└─ WhileDoElse (IsMatchStage: game_progress=4, 0-420s)
   │
   ├─ 【比赛阶段】 ReactiveSequence
   │  │
   │  ├─ CommandHub (SubTree)
   │  │  ├─ DecideEconomyCmd     → {cmd.trig_remote_ammo/hp}, {cmd.allow_ammo_target}
   │  │  ├─ DecideRespawnCmd     → {cmd.confirm_respawn}, {cmd.confirm_instant_respawn}
   │  │  ├─ PostureDegradationGuard: {cmd.posture} → {cmd.final_posture}
   │  │  └─ SentryCmdMux (hz=5) → PUB /sentry_cmd
   │  │
   │  ├─ GlobalRobotControl (ReactiveFallback) → PUB /robot_control
   │  │  ├─ 死亡 → stop_gimbal=T, spin=F
   │  │  ├─ 在补给区 (IsAtGoal, cfg.supply_zone_radius) → stop_gimbal=F, spin=F
   │  │  ├─ 检测到敌人 OR is_at_nav_goal=true → stop_gimbal=F, spin=T
   │  │  └─ 默认 → stop_gimbal=F, spin=F
   │  │
   │  ├─ SurvivalGuard (ReactiveFallback)
   │  │  ├─ HP 正常 + 活着 → Success (放行)
   │  │  └─ 低HP/死亡 → LowHPRetreat (SubTree)
   │  │     ├─ SelectPosture(3=移动)
   │  │     ├─ 死亡 → 停车等复活
   │  │     └─ 低血量 → 导航回补给区 → HoldAndHeal
   │  │
   │  ├─ EnemyDetectionGuard (ReactiveFallback)
   │  │  ├─ 无敌人 → Success (放行)
   │  │  ├─ 目标=补给区 (IsNavTargetSupply) → Success (回家优先)
   │  │  └─ 有敌人 → SelectPosture(1=攻击) + CancelNavGoal + 停车 (KeepRunning)
   │  │
   │  ├─ DefaultPosture (ReactiveFallback) — 全局姿态管理
   │  │  ├─ is_at_nav_goal=true → SelectPosture(2=防御)
   │  │  └─ 否则 → SelectPosture(3=移动)
   │  │
   │  └─ 战术优先级 (ReactiveFallback)
   │     ├─ [0] BaseDefense — 基地受威胁
   │     │   ├─ IsBaseThreatened
   │     │   ├─ SelectPosture(1=攻击)
   │     │   ├─ SendGoal(defend_anchor, hz=1)
   │     │   └─ 到达防御点 + 检测敌人 → 停车对抗
   │     ├─ [1] SustainAndEconomy — 弹药不足时补给
   │     │   ├─ AmmoPlan (SubTree)
   │     │   │  ├─ IsAmmoBelow(ammo_allow, cfg.ammo_low)
   │     │   │  └─ 在补给区→等待配额 / 否则→导航到最近补给站
   │     │   └─ SelectPosture(2=防御) ← 仅 AmmoPlan=RUNNING 时执行
   │     └─ [2] ObjectivePlanner — 目标控制
   │        ├─ SelectObjective(中央/梯形高地)
   │        ├─ ObjectivePatrol(巡逻循环)
   │        └─ SendGoal(hz=1) + KeepRunning
   │
   └─ 【非比赛阶段】 ReactiveSequence
      ├─ RmucRobotControl(stop_gimbal=T, spin=F)
      └─ SentryCmdMux(posture=3, hz=2, 全零)
```

---

## 外部数据源

| 组件 | 类型 | 发布话题 | 说明 |
|---|---|---|---|
| **robot_position_bridge.py** | ROS2 节点 (Python) | `robot_position` | 仿真专用：从 TF (map→base_footprint) + Nav2 action status 合成 RMUCRobotPosition 消息，50Hz |
| **rmuc_test_publisher.py** | 模拟器脚本 | `game_status`, `robot_status`, `rfid_status`, `sentry_decision_status`, `robot_buff`, `projectile_allowance`, `field_status`, `enemy_mark`, `team_hp` | 仿真裁判系统模拟器，订阅 `sentry_cmd` 同步 `current_posture` |

---

## sentry_cmd 与 sentry_decision_status 关系

```
BT 决策 ─→ sentry_cmd (0x0120) ─→ 裁判系统   (命令: "我要求切到防御")
裁判系统 ─→ sentry_decision_status (0x0202) ─→ BT   (反馈: "你当前的姿态是防御")
```

仿真中 `rmuc_test_publisher` 订阅 `sentry_cmd` 并自动同步 `current_posture` 到 `sentry_decision_status`。
