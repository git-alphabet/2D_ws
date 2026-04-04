# 行为树 → 电控 msg 需求清单

> **文档目的**：行为树侧需要以下裁判系统数据来驱动决策。请电控按此清单定义 `.msg` 文件并发布对应话题。
>
> **当前状态**：msg 文件已在 `sp_msgs/msg/RMUC/rmuc_2026` 中定义好，电控侧只需要按照 msg 定义实现串口解析并发布即可。

---

## 话题总览

| # | 话题名 | msg 类型 | 来源协议 | 建议频率 | 紧急度 |
|---|--------|----------|----------|----------|--------|
| 1 | `/sentry_decision_status` | `RMUCSentryDecisionStatus` | `0x020D` | 10 Hz | 🔴 第一批 |
| 2 | `/robot_buff` | `RMUCRobotBuff` | `0x0204` | 10 Hz | 🟡 第二批 |
| 3 | `/projectile_allowance` | `RMUCProjectileAllowance` | `0x0208` | 10 Hz | 🟡 第二批 |
| 4 | `/field_status` | `RMUCFieldStatus` | `0x0101` | 1 Hz | 🟢 第三批 |
| 5 | `/enemy_mark` | `RMUCEnemyMark` | `0x020C` | 10 Hz | 🟢 第三批 |
| 6 | `/team_positions` | `RMUCTeamPositions` | `0x020B` | 1 Hz | 🟢 第三批 |
| 7 | `/team_hp` | `RMUCTeamHP` | `0x0003` | 1 Hz | 🔴 第一批 |

---

## 🔴 第一批（P1 生存闭环必需，最高优先级）

### 1. `/sentry_decision_status` — `RMUCSentryDecisionStatus.msg`

来源协议：**0x020D** 哨兵自主决策信息反馈

```
std_msgs/Header header

bool   can_free_respawn         # bit19: 是否可确认免费复活
bool   can_instant_respawn      # bit20: 是否可兑换立即复活
uint16 instant_respawn_cost     # bit21-30: 立即复活金币数
uint8  current_posture          # bit12-13 (附加): 当前姿态 (1进攻/2防御/3移动)
uint8  remote_ammo_count        # bit11-14: 远程兑换弹量次数
uint8  remote_heal_count        # bit15-18: 远程兑换血量次数
uint16 exchanged_ammo_total     # bit0-10: 累计兑换允许发弹量
bool   can_activate_energy      # 附加 bit14: 能量机关可激活
```

**用途**：`DecideRespawnCmd` 判断复活时机和成本；`DecidePosture` 读取真实姿态反馈。

### 7. `/team_hp` — `RMUCTeamHP.msg`

来源协议：**0x0003** 己方机器人/建筑血量

```
std_msgs/Header header

uint16 hero_hp                 # 英雄血量
uint16 engi_hp                 # 工程血量
uint16 infantry3_hp            # 3号步兵血量
uint16 infantry4_hp            # 4号步兵血量
uint16 sentry_hp               # 哨兵血量
uint16 outpost_hp              # 前哨站血量 (0=被击毁)
uint16 base_hp                 # 基地血量
```

**用途**：前哨站存活判断（`outpost_hp > 0` → 基地无敌）；态势感知。

---

## 🟡 第二批（P2 经济闭环必需）

### 2. `/robot_buff` — `RMUCRobotBuff.msg`

来源协议：**0x0204** 机器人增益

```
std_msgs/Header header

uint8  heal_rate                # 回血增益% (10=10%/s)
uint16 cool_value               # 冷却增益具体值
uint8  defense_pct              # 防御增益%
uint8  vulnerability_pct        # 易伤%
uint16 attack_pct               # 攻击增益%
uint8  remaining_energy         # 剩余能量反馈
```

**用途**：`DecidePosture` 基于真实增益值决策；`HoldAndHeal` 判断回血效率。

### 3. `/projectile_allowance` — `RMUCProjectileAllowance.msg`

来源协议：**0x0208** 允许发弹量与金币

```
std_msgs/Header header

uint16 ammo_17mm               # 17mm 允许发弹量
uint16 ammo_42mm               # 42mm 允许发弹量
uint16 remaining_coins         # 剩余金币
uint16 fortress_ammo           # 堡垒储备 17mm 弹量
```

**用途**：在堡垒时优先消耗储备弹；弹药决策。

---

## 🟢 第三批（P3 目标控制需要）

### 4. `/field_status` — `RMUCFieldStatus.msg`

来源协议：**0x0101** 场地增益点/能量机关状态

```
std_msgs/Header header

bool   supply_no_resource_occupied   # bit0
bool   supply_resource_occupied      # bit1
uint8  small_energy_status           # bit3-4
uint8  big_energy_status             # bit5-6
uint8  central_highland              # bit7-8
uint8  ladder_highland               # bit9-10
uint16 dart_hit_time                 # bit11-19
uint8  dart_hit_target               # bit20-22
uint8  fortress                      # bit25-26
uint8  outpost_buff                  # bit27-28
bool   base_buff                     # bit29
```

**用途**：`SelectObjective` 目标评分；堡垒/高地/前哨站争夺决策。

### 5. `/enemy_mark` — `RMUCEnemyMark.msg`

来源协议：**0x020C** 标记/易伤

```
std_msgs/Header header

bool   enemy_hero_vuln         # bit0
bool   enemy_engi_vuln         # bit1
bool   enemy_infantry3_vuln    # bit2
bool   enemy_infantry4_vuln    # bit3
bool   enemy_sentry_vuln       # bit4
bool   ally_hero_marked        # bit5
bool   ally_engi_marked        # bit6
bool   ally_infantry3_marked   # bit7
bool   ally_infantry4_marked   # bit8
bool   ally_sentry_marked      # bit9
```

**用途**：`SelectBestTarget` 优先攻击易伤目标。

### 6. `/team_positions` — `RMUCTeamPositions.msg`

来源协议：**0x020B** 己方队友位置

```
std_msgs/Header header

float32 hero_x/hero_y
float32 engi_x/engi_y
float32 infantry3_x/infantry3_y
float32 infantry4_x/infantry4_y
```

**用途**：协同防守；避免重复占点。

---

## 命名空间约定

- **仿真**：所有话题带 namespace，如 `/red_standard_robot1/sentry_decision_status`
- **实车**：不带 namespace，如 `/sentry_decision_status`

## 已有话题（无需变更）

| 话题 | msg 类型 | 协议 | 状态 |
|------|----------|------|------|
| `/game_status` | `RMUCGameStatus` | `0x0001` | ✅ 已有 |
| `/robot_status` | `RMUCRobotStatus` | `0x0201` | ✅ 已有 |
| `/rfid_status` | `RMUCRFIDStatus` | `0x0209` | ✅ 已有 |
| `/robot_position` | `RMUCRobotPosition` | `0x0203` | ✅ 已有 |
| `/radar/enemy_tracks` | `RMUCEnemyTracks` | 自定义 | ✅ 已有 |
