# RMUC 2026 哨兵消息规范文档

## 概述

本目录包含 RoboMaster 2026 哨兵自主决策系统的所有核心消息定义，共 **15 个消息类型**：

- **12 个订阅消息**：来自裁判系统、定位系统、雷达等外部数据源
- **3 个发布消息**：从行为树生成的控制指令

所有消息均已根据最新需求调整，**未使用的字段已注释到文件底部**，保持主体结构清晰。

---

## 消息汇总表

### 订阅消息（来自外部 → BT 黑板）

| # | 消息名 | 话题名 | 频率 | 来源 | 主要用途 | BT 订阅节点 |
|---|--------|--------|------|------|---------|-----------|
| 1 | RMUCGameStatus | `/game_status` | 1 Hz | 裁判系统 0x0001 | 比赛阶段控制 | `RmucSubGameStatus` |
| 2 | RMUCRobotStatus | `/robot_status` | 10 Hz | 裁判系统 0x0201 等 | 血量/热量/弹药/死亡 | `RmucSubRobotStatus` |
| 3 | RMUCRFIDStatus | `/rfid_status` | 事件驱动 | 裁判系统 0x0209 | RFID 刷卡检测 | `RmucSubRFIDStatus` |
| 4 | RMUCRobotPosition | `/robot_position` | 50 Hz | 定位系统 LiDAR SLAM | 位姿、导航判定 | `RmucSubRobotPosition` |
| 5 | RMUCEnemyTracks | `/radar/enemy_tracks` | 10-30 Hz | 雷达站 | 敌方目标跟踪 | `SubRadarTracks` |
| 6 | RMUCSentryDecisionStatus | `/sentry_decision_status` | 10 Hz | 裁判系统 0x020D | 复活/姿态/经济反馈 | `RmucSubSentryDecisionStatus` |
| 7 | RMUCRobotBuff | `/robot_buff` | 10 Hz | 裁判系统 0x0204 | 增益状态 | `RmucSubRobotBuff` |
| 8 | RMUCProjectileAllowance | `/projectile_allowance` | 10 Hz | 裁判系统 0x0208 | 允许发弹量/金币 | `RmucSubProjectileAllowance` |
| 9 | RMUCFieldStatus | `/field_status` | 1 Hz | 裁判系统 0x0101 | 增益点/高地状态 | `RmucSubFieldStatus` |
| 10 | RMUCEnemyMark | `/enemy_mark` | 10 Hz | 裁判系统 0x020C | 敌方易伤标记 | `RmucSubEnemyMark` |
| 11 | RMUCTeamPositions | `/team_positions` | 1 Hz | 裁判系统 0x020B | 队友位置 | `RmucSubTeamPositions` |
| 12 | RMUCTeamHP | `/team_hp` | 1 Hz | 裁判系统 0x0003 | 建筑血量 | `RmucSubTeamHP` |

### 发布消息（BT → 电控）

| # | 消息名 | 话题名 | 频率 | 用途 | BT 发布节点 |
|---|--------|--------|------|------|-----------|
| 13 | RMUCSentryCmd | `/sentry_cmd` | 2 Hz | 哨兵自主决策指令 0x0120 | `SentryCmdMux` |
| 14 | RMUCRobotControl | `/robot_control` | 10 Hz | 云台/底盘控制 | `RmucRobotControl` |
| 15 | RMUCNavControlCmd | `/nav_control_cmd` | 按需 | 导航控制 | `RmucNavControlCmd` |

---

## 消息详细定义

### 1. RMUCGameStatus — 比赛状态

**话题**: `/game_status` | **频率**: 1 Hz | **来源**: 裁判系统 0x0001

比赛阶段与剩余时间。`game_progress == 4` 时行为树开始执行战术循环。

| 字段 | 类型 | 说明 |
|------|------|------|
| game_progress | uint8 | 比赛阶段: 0=未开始 1=准备 2=自检 3=5秒倒计时 4=进行中 5=结算 |
| stage_remain_time | uint16 | 当前阶段剩余时间 (秒) |

**引用**: `IsGameTime`, `ParseSentryBlackboard` → 计算 `game.elapsed_s` / `game.remain_s`

---

### 2. RMUCRobotStatus — 机器人综合状态

**话题**: `/robot_status` | **频率**: 10 Hz | **来源**: 裁判系统 0x0201/0x0202/0x0003/0x0101/0x0105

核心战斗状态，驱动所有战术决策。

| 字段 | 类型 | 说明 |
|------|------|------|
| current_hp | uint16 | 当前血量 |
| max_hp | uint16 | 血量上限 |
| shooter_heat | uint16 | 当前枪口热量 |
| ammo_allow | uint16 | 剩余允许发弹量 |
| ammo_left | uint16 | 物理剩余弹丸数 |
| is_dead | bool | 是否战亡 |
| can_remote_heal | bool | 是否可触发远程补血 |
| can_remote_ammo | bool | 是否可触发远程补弹 |
| team_coins | uint16 | 队伍当前金币余额 |
| base_hp_cur | uint16 | 己方基地当前血量 |
| base_hp_max | uint16 | 己方基地血量上限 |
| outpost_alive | bool | 己方前哨站是否存活 |
| is_detect_enemy | bool | 是否检测到敌人 |

**暂未启用**: `heat_limit`, `cooling_rate`, `shooter_power_output`, `can_respawn`, `respawn_countdown_s`

**引用**: `ParseSentryBlackboard` (脱战判定: 连续6s未发射+未被扣血), `IsDead`, `IsHPBelow`, `DecideRespawnCmd`, `WaitAndHeal`

---

### 3. RMUCRFIDStatus — RFID 刷卡检测

**话题**: `/rfid_status` | **频率**: 事件驱动 | **来源**: 裁判系统 0x0209

| 字段 | 类型 | 说明 |
|------|------|------|
| rfid_supply | bool | 补给区增益点 RFID |
| rfid_base_buff | bool | 基地增益点 RFID |
| rfid_outpost_buff | bool | 前哨站增益点 RFID |
| rfid_fortress_enemy | bool | 敌方堡垒 RFID |
| rfid_central_highland | bool | 中央高地 RFID |
| rfid_ladder_highland | bool | 梯形高地 RFID |

**暂未启用**: `rfid_fortress_ally`

**引用**: `IsZoneCardDetected`, `IsAnyDispelCardDetected`, `IsSupplyCardDetected`, `MicroSearchSupplyCard`

---

### 4. RMUCRobotPosition — 位姿与导航判定

**话题**: `/robot_position` | **频率**: 50 Hz | **来源**: 定位系统 LiDAR SLAM

| 字段 | 类型 | 说明 |
|------|------|------|
| pose_x | float32 | 位置 x (meter, map 坐标系) |
| pose_y | float32 | 位置 y (meter, map 坐标系) |
| pose_yaw | float32 | 航向角 (radian) |
| is_at_nav_goal | bool | 是否到达导航目标点 |

**坐标系**: 围挡在红方补给站附近为原点，X 正=向蓝方(长边)，Y 正=向红方停机坪(短边)

**引用**: `SelectBestTarget`, `SelectObjective`, `IsAtGoal`, `IsAtNavGoal`, `WaypointPatrol`, `MoveAround`

---

### 5. RMUCEnemyTracks — 雷达敌方跟踪

**话题**: `/radar/enemy_tracks` | **频率**: 10-30 Hz | **来源**: 雷达站

| 字段 | 类型 | 说明 |
|------|------|------|
| enemy_count | uint8 | 有效跟踪目标数量 (0~7) |
| enemy_x | float32[] | 敌方位置 x 数组 (meter) |
| enemy_y | float32[] | 敌方位置 y 数组 (meter) |

**暂未启用**: `enemy_robot_id`, `enemy_confidence`

**引用**: `ParseSentryBlackboard` → `combat.has_target`, `SelectBestTarget`

---

### 6. RMUCSentryDecisionStatus — 哨兵决策反馈

**话题**: `/sentry_decision_status` | **频率**: 10 Hz | **来源**: 裁判系统 0x020D

裁判系统对哨兵自主命令的实时反馈。

| 字段 | 类型 | 说明 |
|------|------|------|
| can_free_respawn | bool | bit19: 是否可确认免费复活 |
| can_instant_respawn | bool | bit20: 是否可兑换立即复活 |
| instant_respawn_cost | uint16 | bit21-30: 立即复活金币成本 |
| current_posture | uint8 | bit12-13: 当前姿态 (1=攻 2=防 3=移) |
| remote_ammo_count | uint8 | bit11-14: 成功远程兑换弹次数 |
| remote_heal_count | uint8 | bit15-18: 成功远程兑换血次数 |
| exchanged_ammo_total | uint16 | bit0-10: 累计成功兑换发弹量 |

**注释掉的**: `can_activate_energy` (己方能量机关激活状态)

**引用**: `DecideRespawnCmd`, `DecidePosture`

---

### 7. RMUCRobotBuff — 实时增益状态

**话题**: `/robot_buff` | **频率**: 10 Hz | **来源**: 裁判系统 0x0204

| 字段 | 类型 | 说明 |
|------|------|------|
| heal_rate | uint8 | 回血增益% (值10=每秒恢复上限10%) |
| cool_value | uint16 | 冷却增益值 (直接加值, x/s) |
| defense_pct | uint8 | 防御增益% (值50=+50%防御) |
| vulnerability_pct | uint8 | 负防御/易伤% (值30=-30%防御) |
| attack_pct | uint16 | 攻击增益% (值50=+50%伤害) |

**暂未启用**: `remaining_energy`

**引用**: `DecidePosture` (cool_value), `HoldAndHeal` (heal_rate)

---

### 8. RMUCProjectileAllowance — 允许发弹量与金币

**话题**: `/projectile_allowance` | **频率**: 10 Hz | **来源**: 裁判系统 0x0208

| 字段 | 类型 | 说明 |
|------|------|------|
| ammo_17mm | uint16 | 机器人自身 17mm 允许发弹量 |
| remaining_coins | uint16 | 剩余金币数量 |
| fortress_ammo | uint16 | 堡垒储备 17mm 允许发弹量 |

**暂未启用**: `ammo_42mm` (哨兵无 42mm)

**引用**: `DecideEconomyCmd` (单调递增 `allow_ammo_target`), `AmmoPlan`

---

### 9. RMUCFieldStatus — 场地增益点与高地状态

**话题**: `/field_status` | **频率**: 1 Hz | **来源**: 裁判系统 0x0101

| 字段 | 类型 | 说明 |
|------|------|------|
| small_energy_status | uint8 | 小能量机关 (0=未激活 1=已激活 2=激活中) |
| big_energy_status | uint8 | 大能量机关 (0=未激活 1=已激活 2=激活中) |
| central_highland | uint8 | 中央高地 (0=未占 1=己方 2=对方) |
| ladder_highland | uint8 | 梯形高地 (0=未占 1=己方) |
| fortress | uint8 | 堡垒 (0=未占 1=己方 2=对方 3=双方) |
| outpost_buff | uint8 | 前哨站增益点 (0=未占 1=己方 2=对方) |
| base_buff | bool | 基地增益点 (1=已占) |

**暂未启用**: `supply_no_resource_occupied`, `supply_resource_occupied`, `dart_hit_time`, `dart_hit_target`

**引用**: `SelectObjective`, `BaseDefense`, `ObjectivePlanner`

---

### 10. RMUCEnemyMark — 敌方易伤标记

**话题**: `/enemy_mark` | **频率**: 10 Hz | **来源**: 裁判系统 0x020C

标记进度 ≥ 100 时为 true。

| 字段 | 类型 | 说明 |
|------|------|------|
| enemy_hero_vuln | bool | 对方英雄易伤 |
| enemy_engi_vuln | bool | 对方工程易伤 |
| enemy_infantry3_vuln | bool | 对方3号步兵易伤 |
| enemy_infantry4_vuln | bool | 对方4号步兵易伤 |
| enemy_sentry_vuln | bool | 对方哨兵易伤 |

**暂未启用**: `ally_hero_marked`, `ally_engi_marked`, `ally_infantry3_marked`, `ally_infantry4_marked`, `ally_sentry_marked`

**引用**: `ParseSentryBlackboard`, `SelectBestTarget` (优先瞄准易伤目标)

---

### 11. RMUCTeamPositions — 队友位置

**话题**: `/team_positions` | **频率**: 1 Hz | **来源**: 裁判系统 0x020B

坐标系同 RMUCRobotPosition。

| 字段 | 类型 | 说明 |
|------|------|------|
| hero_x | float32 | 英雄 x |
| hero_y | float32 | 英雄 y |
| infantry3_x | float32 | 3号步兵 x |
| infantry3_y | float32 | 3号步兵 y |

**暂未启用**: `engi_x`, `engi_y`, `infantry4_x`, `infantry4_y`

**引用**: `ParseSentryBlackboard` → 协同决策

---

### 12. RMUCTeamHP — 队伍建筑血量

**话题**: `/team_hp` | **频率**: 1 Hz | **来源**: 裁判系统 0x0003

| 字段 | 类型 | 说明 |
|------|------|------|
| outpost_hp | uint16 | 前哨站血量 (0=被击毁) |
| base_hp | uint16 | 基地血量 |

**暂未启用**: `hero_hp`, `engi_hp`, `infantry3_hp`, `infantry4_hp`, `sentry_hp`

**引用**: `ParseSentryBlackboard` → 前哨站存活判断, `BaseDefense`

---

### 13. RMUCSentryCmd — 哨兵自主决策指令（发布）

**话题**: `/sentry_cmd` | **频率**: 2 Hz | **方向**: BT → 电控 (0x0120)

由 `SentryCmdMux` 在 CommandHub 子树中发布。

| 字段 | 类型 | 说明 |
|------|------|------|
| cmd_posture | uint8 | 姿态: 1=进攻 2=防御 3=移动 |
| cmd_confirm_respawn | bool | 确认免费复活 |
| cmd_confirm_instant_respawn | bool | 确认兑换立即复活 |
| cmd_allow_ammo_target | uint16 | 允许发弹量目标值 (单调递增) |
| cmd_trigger_remote_ammo | bool | 触发远程兑换弹 (上升沿有效) |
| cmd_trigger_remote_hp | bool | 触发远程兑换血 (上升沿有效) |

**注释掉的**: `cmd_enable_big_energy`

**发布流程**: `DecidePosture` → posture, `DecideEconomyCmd` → ammo/remote, `DecideRespawnCmd` → respawn → `SentryCmdMux` 聚合发布

---

### 14. RMUCRobotControl — 云台底盘控制（发布）

**话题**: `/robot_control` | **频率**: 10 Hz | **方向**: BT → 电控

| 字段 | 类型 | 说明 |
|------|------|------|
| stop_gimbal_scan | bool | 停止云台扫描 |
| chassis_spin | bool | 底盘小陀螺旋转 |

**使用场景**: `LowHPRetreat` → scan=True 回弹, `BaseDefense` → scan=False 警戒, 非比赛 → scan=True 待命

---

### 15. RMUCNavControlCmd — 导航控制指令（发布）

**话题**: `/nav_control_cmd` | **频率**: 按需 | **方向**: BT → 电控

| 字段 | 类型 | 说明 |
|------|------|------|
| cmd_type | int32 | 0=无操作 1=开始导航 2=终止导航 3=原地不动 |
| emergency_stop | bool | 紧急停止 |

---

## 数据流图

```
╔═══════════════════════════════════════════════════════════════╗
║              裁判系统 + 定位系统 + 雷达站                      ║
╚═══════════════════════════════════════════════════════════════╝
  ↓
【PerceptionAndBlackboard 订阅层】
  RmucSubGameStatus          (1 Hz)
  RmucSubRobotStatus         (10 Hz)
  RmucSubRFIDStatus          (事件)
  RmucSubRobotPosition       (50 Hz)
  SubRadarTracks             (10-30 Hz)
  RmucSubSentryDecisionStatus(10 Hz)
  RmucSubRobotBuff           (10 Hz)
  RmucSubProjectileAllowance (10 Hz)
  RmucSubFieldStatus         (1 Hz)
  RmucSubEnemyMark           (10 Hz)
  RmucSubTeamPositions       (1 Hz)
  RmucSubTeamHP              (1 Hz)
  ↓
╔═══════════════════════════════════════════════════════════════╗
║  黑板 Blackboard                                              ║
║  hp.{cur,max}  heat.cur  ammo.{allow,left}  game.remain_s    ║
║  pose.{x,y,yaw}  combat.has_target  is_dead  outpost.alive   ║
║  enemy.{hero_vuln,...}  sentry.*  buff.*  field.*  ...        ║
╚═══════════════════════════════════════════════════════════════╝
  ↓
【ParseSentryBlackboard — 计算派生字段】
  ↓
╔═══════════════════════════════════════════════════════════════╗
║  战术循环                                                      ║
║  LowHPRetreat → BaseDefense → EngageCombat → SustainEconomy  ║
║                                → ObjectivePlanner             ║
╚═══════════════════════════════════════════════════════════════╝
  ↓
【CommandHub — 汇聚决策】
  DecidePosture    → cmd.posture
  DecideEconomyCmd → cmd.allow_ammo_target / trigger_remote_*
  DecideRespawnCmd → cmd.confirm_*respawn
  ↓
【发布指令】
  RMUCSentryCmd    (2 Hz)  → {posture, ammo_target, respawn, ...}
  RMUCRobotControl (10 Hz) → {stop_gimbal_scan, chassis_spin}
  RMUCNavControlCmd(按需)   → {cmd_type, emergency_stop}
  ↓
╔═══════════════════════════════════════════════════════════════╗
║                  电控通过 CAN/串口 执行                         ║
╚═══════════════════════════════════════════════════════════════╝
```

---

## 仿真命名空间

仿真环境中所有话题添加命名空间前缀 `/red_standard_robot1`：

```bash
# 仿真话题示例
/red_standard_robot1/game_status
/red_standard_robot1/robot_status
/red_standard_robot1/sentry_cmd

# 查看 TF 树 (仿真)
ros2 run rqt_tf_tree rqt_tf_tree --ros-args \
  -r /tf:=tf -r /tf_static:=tf_static \
  -r __ns:=/red_standard_robot1
```

**实车**直接使用话题名，无命名空间前缀。

---

## 暂未启用字段汇总

共 25 个字段分布在 9 个文件中，已注释到各文件底部：

| 文件 | 未启用字段 | 数量 |
|------|-----------|------|
| RMUCRobotStatus | heat_limit, cooling_rate, shooter_power_output, can_respawn, respawn_countdown_s | 5 |
| RMUCEnemyMark | ally_hero_marked ~ ally_sentry_marked | 5 |
| RMUCTeamHP | hero_hp, engi_hp, infantry3_hp, infantry4_hp, sentry_hp | 5 |
| RMUCTeamPositions | engi_x, engi_y, infantry4_x, infantry4_y | 4 |
| RMUCFieldStatus | supply_no_resource_occupied, supply_resource_occupied, dart_hit_time, dart_hit_target | 4 |
| RMUCEnemyTracks | enemy_robot_id, enemy_confidence | 2 |
| RMUCProjectileAllowance | ammo_42mm | 1 |
| RMUCRFIDStatus | rfid_fortress_ally | 1 |
| RMUCRobotBuff | remaining_energy | 1 |

6 个文件全部字段在用（无未启用）：RMUCGameStatus, RMUCRobotPosition, RMUCSentryDecisionStatus, RMUCSentryCmd, RMUCRobotControl, RMUCNavControlCmd

---

## 调试技巧

```bash
# 监听消息
ros2 topic echo /game_status
ros2 topic echo /robot_status
ros2 topic echo /sentry_cmd

# 查询频率
ros2 topic hz /robot_position

# 仿真时加命名空间
ros2 topic echo /game_status --ros-args -r __ns:=/red_standard_robot1
```

---

**文档版本**: 2.0 | **最后更新**: 2026-04-11
