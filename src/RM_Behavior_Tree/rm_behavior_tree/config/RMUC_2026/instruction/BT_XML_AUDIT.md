# BT XML 工程审计文档

> **审计日期**: 2026-04-04  
> **审计范围**: `src/RM_Behavior_Tree/rm_behavior_tree/config/rmuc_2026/` 下全部 16 个 XML 文件  
> **BT 框架**: BT.CPP v4 (`BTCPP_format="4"`)  
> **审计方法**: 控制流分析 + 数据流分析 + 语义分析 + 历史 bug 复盘

---

## 目录

1. [文件清单与层级关系](#1-文件清单与层级关系)
2. [控制流审计：节点类型与返回值传播](#2-控制流审计节点类型与返回值传播)
3. [数据流审计：Blackboard Key 读写配对](#3-数据流审计blackboard-key-读写配对)
4. [Tick 顺序与时序审计](#4-tick-顺序与时序审计)
5. [常见 BT 反模式检查](#5-常见-bt-反模式检查)
6. [TreeNodesModel 一致性审计](#6-treenodesmodel-一致性审计)
7. [已知限制与风险项](#7-已知限制与风险项)
8. [审计方法论总结](#8-审计方法论总结)

---

## 1. 文件清单与层级关系

### 1.1 文件列表

| # | 文件名 | BehaviorTree ID | 角色 |
|---|--------|-----------------|------|
| 1 | `rmuc_2026.xml` | `rmuc_2026` (主树) | 主入口 + TreeNodesModel |
| 2 | `PerceptionAndBlackboard.xml` | `PerceptionAndBlackboard` | 感知订阅 + 黑板解析 |
| 3 | `InitOnce.xml` | `InitOnce` | 配置初始化（幂等） |
| 4 | `CommandHub.xml` | `CommandHub` | 姿态/经济/复活指令决策 |
| 5 | `RespawnRecovery.xml` | `RespawnRecovery` | 死亡等待 + 虚弱恢复（完全重写） |
| 6 | `WeaknessRecovery.xml` | `WeaknessRecovery` | 虚弱安全网（独立路径） |
| 7 | `CriticalSurvival.xml` | `CriticalSurvival` | 危急撤退 |
| 8 | `BaseDefense.xml` | `BaseDefense` | 基地防御 |
| 9 | `EngageCombat.xml` | `EngageCombat` | 交战入口 + 底盘旋转策略（显著增强） |
| 10 | `CombatLoop.xml` | `CombatLoop` | 战斗循环（选目标→瞄准→射击） |
| 11 | `SustainAndEconomy.xml` | `SustainAndEconomy` | 后勤入口 |
| 12 | `HealPlan.xml` | `HealPlan` | 回血计划 |
| 13 | `AmmoPlan.xml` | `AmmoPlan` | 补弹计划 |
| 14 | `ObjectivePlanner.xml` | `ObjectivePlanner` | 目标控制（8候选点评分，显著增强） |
| 15 | `PatrolAndScan.xml` | `PatrolAndScan` | 默认巡逻 |
| 16 | `DeathAndRespawn.xml` | `DeathAndRespawn` | ⚠️ 未被 include，废弃文件 |

### 1.2 调用层级

```
rmuc_2026 (主树) — 外层 ReactiveSequence
├── SubTree: PerceptionAndBlackboard      [每帧 tick]
│   └── 12 个订阅节点 + ParseSentryBlackboard (~70+ 输出)
├── SubTree: InitOnce                     [幂等]
│   └── InitSentryConfig(supply_zone_x/y + 新 recovery 参数) + InitCmdState
└── WhileDoElse (RmucIsGameTime)
    ├── [比赛阶段] ReactiveSequence
    │   ├── SubTree: CommandHub            [每帧 tick]
    │   │   └── DecidePosture → DecideEconomyCmd → DecideRespawnCmd → RateController(5Hz){SentryCmdMux}
    │   └── ReactiveFallback (优先级战术, 8 分支)
    │       ├── Sequence[0]:  SetBlackboard(active_subtree="RespawnRecovery")    + SubTree: RespawnRecovery
    │       ├── Sequence[0.5]: SetBlackboard(active_subtree="WeaknessRecovery")  + SubTree: WeaknessRecovery
    │       ├── Sequence[1]:  SetBlackboard(active_subtree="CriticalSurvival")   + SubTree: CriticalSurvival
    │       ├── Sequence[2]:  SetBlackboard(active_subtree="BaseDefense")        + SubTree: BaseDefense
    │       ├── Sequence[3]:  SetBlackboard(active_subtree="EngageCombat")       + SubTree: EngageCombat
    │       │   └── SubTree: CombatLoop
    │       ├── Sequence[4]:  SetBlackboard(active_subtree="SustainAndEconomy")  + SubTree: SustainAndEconomy
    │       │   ├── SubTree: HealPlan
    │       │   └── SubTree: AmmoPlan
    │       ├── Sequence[5]:  SetBlackboard(active_subtree="ObjectivePlanner")   + SubTree: ObjectivePlanner
    │       └── Sequence[6]:  SetBlackboard(active_subtree="PatrolAndScan")      + SubTree: PatrolAndScan
    └── [非比赛阶段] ReactiveSequence
        └── SendGoal(Home) + RmucRobotControl + SentryCmdMux
```

### 1.3 Include 文件清单（12 个子树）

主树 `rmuc_2026.xml` include 以下 12 个文件：

| 子树文件 | 被引用为 SubTree |
|----------|-----------------|
| PerceptionAndBlackboard.xml | ✅ |
| InitOnce.xml | ✅ |
| CommandHub.xml | ✅ |
| RespawnRecovery.xml | ✅ |
| WeaknessRecovery.xml | ✅ |
| CriticalSurvival.xml | ✅ |
| BaseDefense.xml | ✅ |
| EngageCombat.xml | ✅ |
| CombatLoop.xml | ✅ |
| SustainAndEconomy.xml | ✅ |
| ObjectivePlanner.xml | ✅ |
| PatrolAndScan.xml | ✅ |

> **注意**: `HealPlan.xml` 和 `AmmoPlan.xml` 通过 `SustainAndEconomy.xml` 间接引用，不在主树 include 列表中。

### 1.4 未被引用的文件

| 文件 | 状态 | 说明 |
|------|------|------|
| `DeathAndRespawn.xml` | ⚠️ **未被 include** | 未出现在 `rmuc_2026.xml` 的 `<include>` 列表中，也没有被任何 SubTree 引用。是早期版本的废弃文件，功能已被 `RespawnRecovery.xml` 完全替代。建议删除或标记为废弃。 |

---

## 2. 控制流审计：节点类型与返回值传播

### 2.1 审计规则

| 容器类型 | 子节点 FAILURE | 子节点 RUNNING | 子节点 SUCCESS |
|---------|---------------|---------------|---------------|
| `Sequence` | 立即返回 FAILURE | 返回 RUNNING | 继续下一个 |
| `ReactiveSequence` | 立即返回 FAILURE | 返回 RUNNING, **下帧从头重新 tick** | 继续下一个 |
| `Fallback` | 继续下一个 | 返回 RUNNING | 立即返回 SUCCESS |
| `ReactiveFallback` | 继续下一个 | 返回 RUNNING, **下帧从头重新 tick** | 立即返回 SUCCESS |

### 2.2 逐文件审计

#### rmuc_2026.xml — 主树

```
ReactiveSequence (外层)
├── PerceptionAndBlackboard  → SUCCESS (订阅节点非阻塞)
├── InitOnce                 → SUCCESS (幂等)
└── WhileDoElse
    ├── condition: RmucIsGameTime → SUCCESS/FAILURE
    ├── [true]  ReactiveSequence → 比赛逻辑
    └── [false] ReactiveSequence → 非比赛逻辑
```

- ✅ 外层 `ReactiveSequence`：每帧重新从 PerceptionAndBlackboard 开始 tick，保证感知数据持续更新
- ✅ `WhileDoElse`：条件为 `RmucIsGameTime`，正确区分比赛/非比赛阶段

**比赛阶段内层：**

```
ReactiveSequence
├── CommandHub               → SUCCESS (Decision 节点全同步)
└── ReactiveFallback         → 取决于激活分支
    ├── Sequence[0]: SetBlackboard(active_subtree) + RespawnRecovery
    ├── Sequence[1]: SetBlackboard(active_subtree) + WeaknessRecovery
    ├── ...
    └── Sequence[7]: SetBlackboard(active_subtree) + PatrolAndScan
```

- ✅ `ReactiveSequence` 确保 `CommandHub` 每帧在 ReactiveFallback 之前执行
- ✅ `ReactiveFallback` 每帧从优先级最高的分支开始尝试，保证抢占语义
- ✅ 每个优先级分支用 `Sequence` 包裹 `SetBlackboard` + `SubTree`，写入 `active_subtree` 用于 DecidePosture 任务绑定

#### PerceptionAndBlackboard.xml

```
Sequence
├── RmucSubGameStatus        (/game_status)         → SUCCESS
├── RmucSubRobotStatus       (/robot_status)        → SUCCESS
├── RmucSubRFIDStatus        (/rfid_status)         → SUCCESS
├── RmucSubRobotPosition     (/robot_position)      → SUCCESS
│   outputs: pose_x, pose_y, pose_yaw, is_at_nav_goal, pose
├── SubRadarTracks           (/radar/enemy_tracks)   → SUCCESS
├── RmucSubSentryDecisionStatus (/sentry_decision_status) → SUCCESS  [P0 NEW]
├── RmucSubRobotBuff         (/robot_buff)           → SUCCESS  [P0 NEW]
├── RmucSubProjectileAllowance (/projectile_allowance) → SUCCESS  [P0 NEW]
├── RmucSubFieldStatus       (/field_status)         → SUCCESS  [P0 NEW]
├── RmucSubEnemyMark         (/enemy_mark)           → SUCCESS  [P0 NEW]
├── RmucSubTeamPositions     (/team_positions)       → SUCCESS  [P0 NEW]
└── ParseSentryBlackboard    → SUCCESS (~70+ 输出)
```

- ✅ 当前 live RMUC 树保留已接入的订阅节点 + 1 个解析节点，均为非阻塞 `SyncActionNode`
- ✅ 话题名全部使用绝对路径 (例如 `/game_status`, `/robot_status`)

#### CommandHub.xml

```
Sequence
├── DecidePosture      → SUCCESS (SyncAction)
│   inputs: current_posture, buff_cool_value, buff_defense_pct, buff_vulnerability_pct, ammo_allow, ...
│   outputs: posture_out
├── DecideEconomyCmd   → SUCCESS (SyncAction)
│   inputs: instant_respawn_cost, cumulative_instant_count, base_hp_cur/max, fortress_ammo, remote_heal/ammo_count, ...
├── DecideRespawnCmd   → SUCCESS (SyncAction)
│   inputs: can_free_respawn, can_instant_respawn, instant_respawn_cost, cumulative_instant_count(inout), ...
└── RateController(5Hz)
    └── SentryCmdMux   → SUCCESS (SyncAction)
```

- ✅ 全是 `SyncActionNode`，tick 立即返回 `SUCCESS`
- ✅ `Sequence` 顺序执行，末尾无 `KeepRunning`（已在早期修复中移除）
- ✅ `RateController` 限制 SentryCmdMux 发送频率为 5Hz
- ✅ PosturePublishing 节点已移除
- ℹ️ 文件包含本地 TreeNodesModel（供 Groot2 独立加载）

#### RespawnRecovery.xml — 完全重写

> **重大变更**: 原版使用 `DetectRespawnAndSetRecovery` + `need_recovery` 标志位 + `ClearRecoveryFlag`。
> 新版直接使用 `IsWeakness` 条件节点判断，不再依赖手动标志管理。

```
Fallback (注意：不是 Sequence)
├── Sequence "IfDead_StopAndWait"
│   ├── RmucIsDead           → SUCCESS/FAILURE
│   ├── RmucRobotControl(stop) → SUCCESS
│   └── RmucNavControlCmd(cmd_type=3) → SUCCESS
└── ReactiveSequence "WeaknessRecoveryFlow"
    ├── IsWeakness           → SUCCESS/FAILURE (门控)
    └── Fallback "RecoveryFlow"
        ├── Sequence "IfSupplyCard_Heal"
        │   ├── RmucIsSupplyCardDetected → SUCCESS/FAILURE
        │   └── RmucWaitAndHeal(topic=/robot_status, uses time.now_ms) → RUNNING/SUCCESS
        └── Sequence "GoSupply_ThenSearch"
            ├── RmucNavControlCmd(cmd_type=1)
            ├── ReactiveFallback "NavUntilArrived"
            │   ├── RmucIsAtNavGoal → SUCCESS/FAILURE
            │   └── ReactiveSequence
            │       ├── RateController(5Hz) → SendGoal
            │       └── KeepRunning → RUNNING (保持导航)
            ├── RmucRobotControl (brake)
            ├── InitSearchTimerIfNeeded
            └── ReactiveFallback "DetectOrMicroSearch"
                ├── RmucIsSupplyCardDetected
                └── RmucMicroSearchSupplyCard
```

**状态机语义：**
| 状态 | 行为 | 返回值 |
|------|------|--------|
| 存活 + 不虚弱 | 两个分支都 FAILURE | Fallback → **FAILURE** → 让出给下一优先级 |
| 死亡 | Branch A: 停车 + 导航控制 | Fallback → **SUCCESS** → 独占 ReactiveFallback |
| 存活 + 虚弱 | Branch B: 导航到补给区 + 搜索/治疗 | Fallback → **SUCCESS/RUNNING** → 独占 |

- ✅ 死亡分支：`RmucIsDead` SUCCESS → 停车 → 整个 Fallback 返回 SUCCESS → 独占 ReactiveFallback
- ✅ 虚弱分支：`IsWeakness` SUCCESS → 进入恢复流程；FAILURE → ReactiveSequence FAILURE → Fallback 整体 FAILURE
- ✅ 正常状态（存活+不虚弱）：两个分支都 FAILURE → 整个 Fallback FAILURE → 让出给下一优先级
- ✅ `IsWeakness` 直接读取 `robot_status`，不再依赖手动 `need_recovery` 标志位

#### WeaknessRecovery.xml

```
ReactiveSequence
├── IsWeakness       → SUCCESS/FAILURE (门控)
├── SelectNearestDispelCard → SUCCESS (写 nav.goal_x/y)
│   inputs: supply_zone_x/y, base_buff, outpost_buff
├── RateController(1Hz) → SendGoal
├── RmucRobotControl (fire off)
└── ReactiveFallback
    ├── IsAnyDispelCardDetected → SUCCESS/FAILURE
    └── MoveAround              → RUNNING/SUCCESS
```

- ✅ `IsWeakness` FAILURE → 整棵树立即退出
- ✅ 使用 `supply_zone_x/y`（非旧版 `supply_x/y`）
- ✅ 到达后 `MoveAround` 微动搜索，直到检测到解除卡
- ✅ 关闭射击（fire off），避免虚弱期浪费弹药

#### CriticalSurvival.xml

```
ReactiveSequence
├── IsCriticalState    → SUCCESS/FAILURE (门控)
├── RmucRobotControl   → SUCCESS
├── SelectSafeRetreatGoal → SUCCESS (使用 supply_zone_x/y)
├── RateController(1Hz) → SendGoal(Retreat)
└── KeepRunning        → RUNNING (保持)
```

- ✅ `IsCriticalState` FAILURE → 退出，让出给 BaseDefense
- ✅ `KeepRunning` 保持 RUNNING → ReactiveSequence 每帧重新检查门控
- ✅ **无 CancelNavGoal**（已移除 — 在 ReactiveSequence 中会导致每帧取消导航）
- ✅ `SelectSafeRetreatGoal` 使用 `supply_zone_x/y`（非旧版 `supply_x/y`）

#### BaseDefense.xml

```
ReactiveSequence
├── IsBaseThreatened   → SUCCESS/FAILURE (门控)
│   inputs: 包含 outpost_alive (新增)
├── RmucRobotControl   → SUCCESS (fire_enable=True)
├── RateController(1Hz) → SendGoal (defend_anchor)
└── SubTree: CombatLoop → RUNNING
```

- ✅ 威胁解除 → `IsBaseThreatened` FAILURE → 退出
- ✅ 防御时导航到锚点 + 同时战斗
- ✅ `IsBaseThreatened` 新增 `outpost_alive` 输入，前哨站存活时降低基地威胁判定

#### EngageCombat.xml — 显著增强

```
ReactiveSequence
├── HasValidTarget      → SUCCESS/FAILURE
├── IsCombatAllowed     → SUCCESS/FAILURE (使用 robot_status，非旧版 is_weak)
├── SetBlackboard (nav.goal_x = pose.x)      [NEW]
├── SetBlackboard (nav.goal_y = pose.y)      [NEW]
├── ReactiveFallback (旋转策略)               [NEW]
│   ├── Sequence: ShouldChassisSpin → RmucRobotControl(spin=True, fire=True)
│   └── RmucRobotControl(spin=False, fire=True)
└── SubTree: CombatLoop → RUNNING
```

- ✅ 目标丢失/战斗不允许 → FAILURE → 退出交战
- ✅ `IsCombatAllowed` 读取 `robot_status`（非旧版 `is_weak`），直接判断虚弱状态
- ✅ `SetBlackboard` 覆写 nav.goal 为当前位置 → ShouldChassisSpin 计算 dist≈0 → 允许旋转
- ✅ 退出后下帧 ObjectivePlanner/PatrolAndScan 会重新写入正确 nav.goal
- ✅ `ShouldChassisSpin`：检查距离/姿态/功率提升 → 满足条件时启用底盘旋转
- ✅ 无论旋转与否，两个分支都启用射击 (fire=True)
- ℹ️ 文件包含本地 TreeNodesModel（供 Groot2 独立加载）

#### CombatLoop.xml

```
ReactiveSequence
├── SelectBestTarget    → SUCCESS
│   inputs: 新增 enemy_hero/engi/infantry3/infantry4/sentry_vuln
├── AimAtTarget         → SUCCESS
├── ReactiveFallback (射击窗口)
│   ├── IsFireWindowOk  → SUCCESS/FAILURE
│   │   inputs: 新增 robot_status, current_posture, buff_cool_value, buff_vulnerability_pct
│   │   inputs: 移除 is_weak
│   └── RmucRobotControl (fire_enable=False)
├── FireBurst           → RUNNING/SUCCESS
└── KeepRunning         → RUNNING
```

- ✅ `IsFireWindowOk` FAILURE → RmucRobotControl 关火 → ReactiveFallback 返回 SUCCESS → 继续循环
- ✅ `FireBurst` RUNNING 期间 → ReactiveSequence 持续重检 SelectBestTarget → 目标实时更新
- ✅ `SelectBestTarget` 新增敌方脆弱度输入，支持优先攻击脆弱目标
- ✅ `IsFireWindowOk` 不再使用 `is_weak`，改用 `robot_status` + `current_posture` + `buff_cool_value` + `buff_vulnerability_pct`

#### SustainAndEconomy.xml

```
ReactiveFallback
├── SubTree: HealPlan   → FAILURE/RUNNING/SUCCESS
└── SubTree: AmmoPlan   → FAILURE/RUNNING/SUCCESS
```

- ✅ 血量充足 → HealPlan FAILURE → 尝试 AmmoPlan
- ✅ 弹量充足 → AmmoPlan FAILURE → 整个 SustainAndEconomy FAILURE → 让出
- ⚠️ **注意**: HealPlan 和 AmmoPlan 都可能导航到补给区，但目标可能不同（supply_zone vs nearest_station）

#### HealPlan.xml

```
ReactiveSequence
├── RmucIsHPBelow      → SUCCESS/FAILURE (门控)
└── ReactiveFallback
    ├── ReactiveSequence (已在补给区: hold+heal)
    │   ├── IsZoneCardDetected(SUPPLY)
    │   └── HoldAndHeal
    └── ReactiveSequence (未在补给区: 导航)
        ├── RateController(1Hz) → SendGoal (supply_zone)
        ├── RmucRobotControl
        └── KeepRunning
```

- ✅ 回血到安全值 → `RmucIsHPBelow` FAILURE → 退出

#### AmmoPlan.xml

```
ReactiveSequence
├── IsAmmoBelow        → SUCCESS/FAILURE (门控)
└── ReactiveFallback
    ├── ReactiveSequence (已在补给区: hold+tick)
    │   ├── IsZoneCardDetected(SUPPLY)
    │   └── HoldForSupplyAmmoTick
    └── ReactiveSequence (未在补给区: 导航)
        ├── SelectNearestResupplyStation
        ├── RateController(1Hz) → SendGoal
        ├── RmucRobotControl
        └── KeepRunning
```

- ✅ 弹量充足 → `IsAmmoBelow` FAILURE → 退出

#### ObjectivePlanner.xml — 显著增强

```
ReactiveSequence
├── SelectObjective    → SUCCESS (永远成功, 写 nav.goal_x/y)
│   inputs (~30+): 新增 ammo_allow, ammo_target,
│     field_central_highland, field_ladder_highland, field_fortress,
│     field_outpost_buff, field_base_buff,
│     fortress_ammo,
│     8 个候选点坐标 (supply_zone_x/y, central_highland_x/y, ...)
├── RateController(1Hz) → SendGoal
├── RmucRobotControl
├── ReactiveFallback
│   ├── IsAtGoal       → SUCCESS/FAILURE
│   └── KeepRunning    → RUNNING
└── HoldObjective      → SUCCESS/RUNNING/FAILURE
```

- ✅ `SelectObjective` 永远返回 SUCCESS → 这棵树**永远被激活**（作为优先级 [5] 的安全网）
- ✅ 如果更高优先级分支都 FAILURE，`ObjectivePlanner` 一定能接住
- ✅ 新增场地状态 (`field_*`) 和堡垒弹量 (`fortress_ammo`) 输入，支持更精细的目标评分
- ⚠️ **注意**: `HoldObjective` 内部有 `base_threat`/`has_target` 提前打断机制

#### PatrolAndScan.xml

```
ReactiveSequence (原为 Sequence，已改为 ReactiveSequence)
├── RmucRobotControl   → SUCCESS (all False)
├── WaypointPatrol     → SUCCESS (写 nav.goal)
├── RateController(1Hz) → SendGoal
└── KeepRunning        → RUNNING
```

- ✅ 兜底分支，永远 RUNNING
- ✅ `WaypointPatrol` 每帧选择下一个路点
- ✅ 容器为 `ReactiveSequence`，保证每帧重新 tick WaypointPatrol

#### InitOnce.xml

```
Sequence (幂等)
├── InitSentryConfig → SUCCESS
│   使用 supply_zone_x/y（非旧版 supply_x/y）
│   新增配置参数: supply_zone_x/y, heal_wait_ms, heal_min_ratio, search_timeout_ms
└── InitCmdState     → SUCCESS
```

- ✅ 全部配置参数使用 `supply_zone_x/y` 命名
- ✅ 新增 RespawnRecovery 相关配置参数

---

## 3. 数据流审计：Blackboard Key 读写配对

### 3.1 审计方法

对于每一个在 XML 中被 `{key}` 引用的 blackboard key：
1. 找到**所有写入源**（`output_port` / `inout_port` / `SetBlackboard`）
2. 找到**所有读取方**（`input_port` / `inout_port`）
3. 检查是否**有读无写**（悬空输入）或**有写无读**（死数据）

### 3.2 感知层写入（数据源 — 12 个订阅节点）

| Blackboard Key | 写入节点 | 话题 | 新增标记 |
|---|---|---|---|
| `{game_status}` | `RmucSubGameStatus` | `/game_status` | |
| `{robot_status}` | `RmucSubRobotStatus` | `/robot_status` | |
| `{rfid.status}` | `RmucSubRFIDStatus` | `/rfid_status` | |
| `{pose.x}`, `{pose.y}`, `{pose.yaw}` | `RmucSubRobotPosition` | `/robot_position` | |
| `{is_at_nav_goal}` | `RmucSubRobotPosition` | `/robot_position` | |
| `{pose}` | `RmucSubRobotPosition` | `/robot_position` | (TransformStamped) |
| `{radar.tracks}` | `SubRadarTracks` | `/radar/enemy_tracks` | |
| `{time.now_ms}` | `RmucSubGameStatus` | `/game_status` | |
| `{sentry_decision_status}` | `RmucSubSentryDecisionStatus` | `/sentry_decision_status` | P0 NEW |
| `{robot_buff}` | `RmucSubRobotBuff` | `/robot_buff` | P0 NEW |
| `{projectile_allowance}` | `RmucSubProjectileAllowance` | `/projectile_allowance` | P0 NEW |
| `{field_status}` | `RmucSubFieldStatus` | `/field_status` | P0 NEW |
| `{enemy_mark}` | `RmucSubEnemyMark` | `/enemy_mark` | P0 NEW |
| `{team_positions}` | `RmucSubTeamPositions` | `/team_positions` | P0 NEW |

### 3.3 ParseSentryBlackboard 输出（二级衍生数据，~70+ key）

#### 原有输出

| Blackboard Key | 读取方 | 状态 |
|---|---|---|
| `{game.remain_s}` | DecideEconomyCmd, DecideRespawnCmd | ✅ |
| `{game.elapsed_s}` | DecidePosture, SelectObjective, HoldAndHeal, HoldForSupplyAmmoTick | ✅ |
| `{hp.cur}` | DecidePosture, DecideEconomyCmd, IsCriticalState, HoldAndHeal, CombatLoop 等 | ✅ |
| `{hp.max}` | DecidePosture, DecideEconomyCmd, HoldAndHeal, RmucWaitAndHeal | ✅ |
| `{heat.cur}` | DecidePosture, IsCriticalState, IsCombatAllowed, IsFireWindowOk | ✅ |
| `{ammo.allow}` | DecidePosture, DecideEconomyCmd, IsCombatAllowed, IsAmmoBelow 等 | ✅ |
| `{ammo.left}` | — | ⚠️ **有写无读** |
| `{base.hp.cur}` | DecideEconomyCmd, DecideRespawnCmd, IsBaseThreatened, SelectObjective | ✅ |
| `{base.hp.max}` | DecideEconomyCmd, DecideRespawnCmd, IsBaseThreatened, SelectObjective | ✅ |
| `{outpost.alive}` | IsBaseThreatened, SelectObjective | ✅ |
| `{state.is_dead}` | DecideRespawnCmd | ✅ |
| `{state.disengaged}` | DecidePosture, DecideEconomyCmd, HoldAndHeal | ✅ |
| `{state.disengage_cd_s}` | — | ⚠️ **有写无读** |
| `{economy.can_remote_heal}` | DecideEconomyCmd | ✅ |
| `{economy.can_remote_ammo}` | DecideEconomyCmd | ✅ |
| `{economy.coins}` | DecideEconomyCmd, DecideRespawnCmd | ✅ |
| `{combat.has_target}` | DecidePosture, HasValidTarget, HoldObjective | ✅ |
| `{combat.best_target}` | HasValidTarget, AimAtTarget, SelectBestTarget(output) | ✅ |
| `{threat.base}` | DecidePosture, DecideEconomyCmd, DecideRespawnCmd, IsBaseThreatened, HoldObjective, SelectObjective | ✅ |
| `{threat.fortress}` | — | ⚠️ **有写无读** |

#### P0 NEW — 0x020D 哨兵决策状态

| Blackboard Key | 读取方 | 状态 |
|---|---|---|
| `{sentry.can_free_respawn}` | DecideRespawnCmd | ✅ |
| `{sentry.can_instant_respawn}` | DecideRespawnCmd | ✅ |
| `{sentry.instant_respawn_cost}` | DecideEconomyCmd, DecideRespawnCmd | ✅ |
| `{sentry.current_posture}` | DecidePosture, ShouldChassisSpin, IsFireWindowOk | ✅ |
| `{sentry.remote_ammo_count}` | DecideEconomyCmd | ✅ |
| `{sentry.remote_heal_count}` | DecideEconomyCmd | ✅ |
| `{sentry.exchanged_ammo_total}` | — | ⚠️ **有写无读** |
| `{sentry.can_activate_energy}` | — | ⚠️ **有写无读** |

#### P0 NEW — 0x0204 机器人增益

| Blackboard Key | 读取方 | 状态 |
|---|---|---|
| `{buff.heal_rate}` | — | ⚠️ **有写无读** |
| `{buff.cool_value}` | DecidePosture, IsFireWindowOk | ✅ |
| `{buff.defense_pct}` | DecidePosture | ✅ |
| `{buff.vulnerability_pct}` | DecidePosture, IsFireWindowOk | ✅ |
| `{buff.attack_pct}` | — | ⚠️ **有写无读** |

#### P0 NEW — 0x0208 堡垒弹量

| Blackboard Key | 读取方 | 状态 |
|---|---|---|
| `{economy.fortress_ammo}` | DecideEconomyCmd, SelectObjective | ✅ |

#### P0 NEW — 0x0101 场地状态

| Blackboard Key | 读取方 | 状态 |
|---|---|---|
| `{field.central_highland}` | SelectObjective | ✅ |
| `{field.ladder_highland}` | SelectObjective | ✅ |
| `{field.fortress}` | SelectObjective | ✅ |
| `{field.outpost_buff}` | SelectObjective | ✅ |
| `{field.base_buff}` | SelectObjective | ✅ |
| `{field.small_energy}` | — | ⚠️ **有写无读** |
| `{field.big_energy}` | — | ⚠️ **有写无读** |

#### P0 NEW — 0x020C 敌方脆弱度

| Blackboard Key | 读取方 | 状态 |
|---|---|---|
| `{enemy.hero_vuln}` | SelectBestTarget | ✅ |
| `{enemy.engi_vuln}` | SelectBestTarget | ✅ |
| `{enemy.infantry3_vuln}` | SelectBestTarget | ✅ |
| `{enemy.infantry4_vuln}` | SelectBestTarget | ✅ |
| `{enemy.sentry_vuln}` | SelectBestTarget | ✅ |

#### P0 NEW — 0x0003 队伍血量

| Blackboard Key | 读取方 | 状态 |
|---|---|---|
| `{team.outpost_hp}` | — | ⚠️ **有写无读** |
| `{team.base_hp}` | — | ⚠️ **有写无读** |

#### P1 NEW — 复活/功率增强状态

| Blackboard Key | 读取方 | 状态 |
|---|---|---|
| `{state.respawn_invincible}` | — | ⚠️ **有写无读** |
| `{state.respawn_invincible_remain_s}` | — | ⚠️ **有写无读** |
| `{state.power_boosted}` | ShouldChassisSpin | ✅ |
| `{state.power_boost_remain_s}` | — | ⚠️ **有写无读** |

#### P1 NEW — 累计即时复活次数

| Blackboard Key | 读取方 | 状态 |
|---|---|---|
| `{respawn.cum_instant_count}` | DecideEconomyCmd, DecideRespawnCmd, ParseSentryBlackboard(inout) | ✅ |

#### 重要说明

> **`is_weak` 已移除**: `is_weak` 不再是 `ParseSentryBlackboard` 的输出。虚弱状态现在通过 `IsWeakness` 条件节点直接读取 `robot_status` 判断。`IsCombatAllowed` 和 `IsFireWindowOk` 也改为读取 `robot_status` 而非 `is_weak`。

### 3.4 配置层写入（InitSentryConfig）

| Blackboard Key 前缀 | 示例 | 读取方 | 状态 |
|---|---|---|---|
| `{cfg.home_x/y}` | 非比赛阶段 SendGoal | ✅ |
| `{cfg.supply_zone_x/y}` | HealPlan, SelectNearestDispelCard, CriticalSurvival 等 | ✅ |
| `{cfg.base_buff_x/y}` | SelectNearestDispelCard 等 | ✅ |
| `{cfg.outpost_buff_x/y}` | SelectNearestDispelCard 等 | ✅ |
| `{cfg.fortress_enemy_x/y}` | SelectObjective | ✅ |
| `{cfg.central_highland_x/y}` | SelectObjective | ✅ |
| `{cfg.ladder_highland_x/y}` | SelectObjective | ✅ |
| `{cfg.defend_anchor_x/y}` | BaseDefense, CriticalSurvival, SelectObjective | ✅ |
| `{cfg.patrol.0~2.x/y}` | WaypointPatrol | ✅ |
| `{cfg.arrive_radius}` | ShouldChassisSpin, IsAtGoal | ✅ |
| `{cfg.hp_critical}` | IsCriticalState | ✅ |
| `{cfg.hp_low}` | IsCombatAllowed, RmucIsHPBelow | ✅ |
| `{cfg.hp_safe}` | HoldAndHeal | ✅ |
| `{cfg.heat_high}` | DecidePosture, IsCombatAllowed, IsFireWindowOk | ✅ |
| `{cfg.heat_critical}` | IsCriticalState | ✅ |
| `{cfg.ammo_low}` | DecideEconomyCmd, IsAmmoBelow | ✅ |
| `{cfg.ammo_target}` | DecideEconomyCmd, HoldForSupplyAmmoTick, SelectObjective | ✅ |
| `{cfg.supply_zone_x/y}` | RespawnRecovery SendGoal | ✅ (NEW) |
| `{cfg.heal_wait_ms}` | RmucWaitAndHeal | ✅ (NEW) |
| `{cfg.heal_min_ratio}` | RmucWaitAndHeal | ✅ (NEW) |
| `{cfg.search_timeout_ms}` | RmucMicroSearchSupplyCard | ✅ (NEW) |
| `{cfg.base_deficit_for_fortress}` | SelectObjective | ✅ |
| `{cfg.enemy_near_base_radius}` | IsBaseThreatened | ✅ |
| `{cfg.objective_hold_ms}` | HoldObjective | ✅ |
| `{cfg.combat_fire_burst_ms}` | FireBurst | ✅ |
| `{cfg.combat_fire_pause_ms}` | FireBurst | ✅ |

### 3.5 运行时动态 Key

| Key | 写入方 | 读取方 | 状态 |
|---|---|---|---|
| `{nav.goal_x}` / `{nav.goal_y}` | SelectObjective, WaypointPatrol, SelectNearestDispelCard, SelectNearestResupplyStation, SelectSafeRetreatGoal, EngageCombat(SetBlackboard) | SendGoal(各处), ShouldChassisSpin, IsAtGoal | ✅ |
| `{nav.objective}` | SelectObjective | ObjectivePlanner(SendGoal name) | ✅ |
| `{cmd.posture}` | DecidePosture | SentryCmdMux (在 CommandHub) | ✅ |
| `{cmd.confirm_respawn}` | DecideRespawnCmd | SentryCmdMux | ✅ |
| `{cmd.confirm_instant_respawn}` | DecideRespawnCmd | SentryCmdMux | ✅ |
| `{cmd.allow_ammo_target}` | DecideEconomyCmd(inout) | SentryCmdMux | ✅ |
| `{cmd.trig_remote_ammo}` | DecideEconomyCmd | SentryCmdMux | ✅ |
| `{cmd.trig_remote_hp}` | DecideEconomyCmd | SentryCmdMux | ✅ |
| `{cmd.enable_big_energy}` | DecideEconomyCmd | SentryCmdMux | ✅ |
| `{cmd.state}` | InitCmdState(inout), SentryCmdMux(inout) | — | ✅ |
| `{active_subtree}` | SetBlackboard (rmuc_2026.xml 各分支, 共 8 处) | DecidePosture | ✅ |
| `{heal_start_ms}` | RmucWaitAndHeal(inout) | RmucWaitAndHeal(inout) | ✅ (子树内部闭环) |
| `{search_start_ms}` | InitSearchTimerIfNeeded(inout) | RmucMicroSearchSupplyCard(inout) | ✅ |
| `{pose}` | RmucSubRobotPosition (TransformStamped output) | MoveAround(message) | ✅ |

### 3.6 有写无读汇总（Dead Data）

以下 key 由 `ParseSentryBlackboard` 输出但从未被任何节点读取：

| Key | 说明 | 风险等级 |
|---|---|---|
| `{ammo.left}` | 剩余弹量（区别于允许弹量 ammo.allow） | 🟡 低 — 可能是预留字段 |
| `{state.disengage_cd_s}` | 脱战倒计时秒数 | 🟡 低 — 可能用于日志/UI |
| `{threat.fortress}` | 堡垒威胁 | 🟡 低 — 可能后续版本使用 |
| `{sentry.exchanged_ammo_total}` | 已兑换弹量总量 | 🟡 低 |
| `{sentry.can_activate_energy}` | 能否激活大能量机关 | 🟡 低 |
| `{buff.heal_rate}` | 回血增益速率 | 🟡 低 |
| `{buff.attack_pct}` | 攻击增益百分比 | 🟡 低 |
| `{field.small_energy}` | 小能量机关状态 | 🟡 低 |
| `{field.big_energy}` | 大能量机关状态 | 🟡 低 |
| `{team.outpost_hp}` | 己方前哨站血量 | 🟡 低 |
| `{team.base_hp}` | 己方基地血量 | 🟡 低 |
| `{state.respawn_invincible}` | 复活无敌状态 | 🟡 低 |
| `{state.respawn_invincible_remain_s}` | 复活无敌剩余秒数 | 🟡 低 |
| `{state.power_boost_remain_s}` | 功率提升剩余秒数 | 🟡 低 |

> **评估**: 共 14 个有写无读 key。这些是 P0/P1 阶段预先解析的数据，为后续优化预留。不影响当前功能，不算 bug。

### 3.7 关键命名变更

| 旧名称 | 新名称 | 影响范围 |
|--------|--------|---------|
| `supply_x` / `supply_y` | `supply_zone_x` / `supply_zone_y` | 全部 Select* 节点 + InitSentryConfig |
| `is_weak` (黑板 key) | 移除，改用 `IsWeakness` 条件节点 | IsCombatAllowed, IsFireWindowOk, RespawnRecovery |

---

## 4. Tick 顺序与时序审计

### 4.1 主树每帧 Tick 顺序

```
帧 N:
  1. PerceptionAndBlackboard → Sub 节点 + ParseSentryBlackboard
     写入: game_status, robot_status, hp.cur, hp.max, heat.cur, ammo.allow, ...
     写入(P0 NEW): sentry_decision_status, robot_buff, projectile_allowance, field_status, enemy_mark, team_positions
     写入(P1 NEW): respawn_invincible, power_boosted, cum_instant_count, ...
  2. InitOnce → 跳过（已执行）
  3. WhileDoElse → RmucIsGameTime
  4. [比赛] ReactiveSequence:
     4a. CommandHub → DecidePosture 读取 {active_subtree}
         ⚠️ 此时 active_subtree 是帧 N-1 写入的值
     4b. ReactiveFallback → SetBlackboard 写入 {active_subtree}
         然后 tick 激活的 SubTree
```

### 4.2 已知时序问题

| 问题 | 影响 | 严重度 |
|---|---|---|
| `active_subtree` 有一帧延迟 | DecidePosture 读到上一帧的任务名 | 🟢 **极低** — 姿态切换有 5s 硬冷却，一帧延迟 (~10-100ms) 不会导致错误切换 |
| 第一帧 `active_subtree` 为空 | 走兜底分支（状态推断） | 🟢 **无影响** — cpp 已处理空字符串场景 |

### 4.3 导航目标竞争分析

多个子树都会写 `{nav.goal_x/y}`：

| 子树 | 写入时机 | 竞争风险 |
|---|---|---|
| EngageCombat | `SetBlackboard` (pose.x/y) | 仅交战时 |
| SelectObjective | 每帧 tick 时 | 仅 ObjectivePlanner 激活时 |
| WaypointPatrol | 每帧 tick 时 | 仅 PatrolAndScan 激活时 |
| SelectNearestDispelCard | 每帧 tick 时 | 仅 WeaknessRecovery 激活时 |
| SelectSafeRetreatGoal | 每帧 tick 时 | 仅 CriticalSurvival 激活时 |
| SelectNearestResupplyStation | 每帧 tick 时 | 仅 AmmoPlan 激活时 |

- ✅ **不存在竞争**: `ReactiveFallback` 确保同一帧只有一个战术子树激活，`nav.goal` 只被一个写入方控制
- ⚠️ **但** `CommandHub` 中 `ShouldChassisSpin` 读取的 `nav.goal` 是**上一帧**的值（与 active_subtree 相同的时序延迟）

### 4.4 RateController 频率汇总

| 节点 | 频率 | 位置 | 说明 |
|---|---|---|---|
| SentryCmdMux | 5Hz | CommandHub.xml | 指令发送频率 |
| SendGoal (各处) | 1Hz | CriticalSurvival, BaseDefense, ObjectivePlanner, PatrolAndScan, WeaknessRecovery, HealPlan, AmmoPlan | 导航目标发送频率 |
| SendGoal (RespawnRecovery) | 5Hz | RespawnRecovery.xml NavUntilArrived | 紧急导航较高频率 |

---

## 5. 常见 BT 反模式检查

### 5.1 已修复的反模式（历史轮次）

| 反模式 | 原始位置 | 修复方式 | 修复轮次 |
|---|---|---|---|
| `KeepRunning` 阻塞 Sequence | CommandHub.xml | 移除 KeepRunning | Round 1 |
| `Sequence` 不响应条件变化 | rmuc_2026.xml 外层 | 改为 ReactiveSequence | Round 2 |
| `CancelNavGoal` 在 ReactiveSequence 中 | CriticalSurvival.xml | 移除 CancelNavGoal | Round 3 |
| `Sequence` 不重新 tick 巡逻 | PatrolAndScan.xml | 改为 ReactiveSequence | Round 3 |
| `need_recovery` 手动标志管理 | RespawnRecovery.xml | 改用 IsWeakness 直接判断 | Round 4 (完全重写) |

### 5.2 当前检查清单

| 检查项 | 结果 | 说明 |
|---|---|---|
| ReactiveSequence 中是否有会阻塞的非门控节点 | ✅ 通过 | 所有 ReactiveSequence 的第一个子节点都是条件/门控 |
| ReactiveSequence 中是否有 CancelNavGoal | ✅ 通过 | 已在 Round 3 移除 |
| Fallback 子节点是否都能正确返回 FAILURE | ✅ 通过 | 每个门控 FAILURE → 子树 FAILURE → Fallback 继续 |
| KeepRunning 是否用在了正确的位置 | ✅ 通过 | 仅在 ReactiveSequence 最后一个子节点 |
| SetBlackboard 是否在 Sequence 中不会阻塞 | ✅ 通过 | SetBlackboard 永远返回 SUCCESS |
| SubTree _autoremap 是否一致 | ✅ 通过 | 所有 SubTree 都使用 _autoremap="true" |
| RateController 内是否有多个子节点 | ✅ 通过 | 每个 RateController 只有 1 个子节点 |
| 是否存在手动标志位管理 | ✅ 通过 | RespawnRecovery 已改用 IsWeakness 条件节点 |

### 5.3 潜在风险模式

| 模式 | 位置 | 风险 | 建议 |
|---|---|---|---|
| Sequence 包裹 SetBlackboard + SubTree (8处) | rmuc_2026.xml ReactiveFallback 内 | 🟢 极低 — SetBlackboard 永远 SUCCESS | 保持现状 |
| `MoveAround` 使用 `{pose}` key | WeaknessRecovery.xml | 🟢 **已确认** — `{pose}` 由 RmucSubRobotPosition 写入 (TransformStamped) | 无需修改 |
| RespawnRecovery 导航到补给区后搜索 | RespawnRecovery.xml | 🟡 中等 — 若搜索超时可能卡住 | 已有 search_timeout_ms 配置兜底 |

---

## 6. TreeNodesModel 一致性审计

### 6.1 审计规则

- 每个在 XML 中使用的自定义节点，必须在 `rmuc_2026.xml` 的 `<TreeNodesModel>` 中有对应定义
- port 名称和方向（input/output/inout）必须与 C++ `providedPorts()` 一致
- `default` 值不影响运行时（仅供 Groot2 显示），但应与实际用法一致

### 6.2 订阅节点覆盖（12 个 Action 节点）

| XML 使用的节点 | TreeNodesModel 定义 | 状态 |
|---|---|---|
| `RmucSubGameStatus` | ✅ | |
| `RmucSubRobotStatus` | ✅ | |
| `RmucSubRFIDStatus` | ✅ | |
| `RmucSubRobotPosition` | ✅ (含 pose output) | |
| `SubRadarTracks` | ✅ | |
| `RmucSubSentryDecisionStatus` | ✅ | P0 NEW |
| `RmucSubRobotBuff` | ✅ | P0 NEW |
| `RmucSubProjectileAllowance` | ✅ | P0 NEW |
| `RmucSubFieldStatus` | ✅ | P0 NEW |
| `RmucSubEnemyMark` | ✅ | P0 NEW |
| `RmucSubTeamPositions` | ✅ | P0 NEW |

### 6.3 配置/解析节点

| XML 使用的节点 | TreeNodesModel 定义 | 关键变更 |
|---|---|---|
| `InitSentryConfig` | ✅ | supply_zone_x/y + supply_zone_x/y + heal_wait_ms + heal_min_ratio + search_timeout_ms |
| `InitCmdState` | ✅ | |
| `ParseSentryBlackboard` | ✅ | 7 个新 input (P0 消息) + ~40 个新 output |

### 6.4 决策/指令节点

| XML 使用的节点 | TreeNodesModel 定义 | 关键变更 |
|---|---|---|
| `DecidePosture` | ✅ | 新增 current_posture, buff_cool_value, buff_defense_pct, buff_vulnerability_pct, ammo_allow 输入 |
| `DecideEconomyCmd` | ✅ | 新增 instant_respawn_cost, cumulative_instant_count, base_hp_cur/max, fortress_ammo, remote_heal/ammo_count 输入 |
| `DecideRespawnCmd` | ✅ | 新增 can_free_respawn, can_instant_respawn, instant_respawn_cost, cumulative_instant_count(inout) 输入 |
| `SentryCmdMux` | ✅ | |

### 6.5 导航/控制节点

| XML 使用的节点 | TreeNodesModel 定义 | 状态 |
|---|---|---|
| `SendGoal` | ✅ | |
| `MoveAround` | ✅ | |
| `KeepRunning` | ✅ | |
| `RmucRobotControl` | ✅ | |
| `RmucNavControlCmd` | ✅ | NEW |
| `RmucWaitAndHeal` | ✅ | NEW |
| `InitSearchTimerIfNeeded` | ✅ | NEW |
| `RmucMicroSearchSupplyCard` | ✅ | NEW |

### 6.6 战斗节点

| XML 使用的节点 | TreeNodesModel 定义 | 关键变更 |
|---|---|---|
| `SelectBestTarget` | ✅ | 新增 enemy_hero/engi/infantry3/infantry4/sentry_vuln 输入 |
| `AimAtTarget` | ✅ | |
| `FireBurst` | ✅ | |
| `IsFireWindowOk` | ✅ | 新增 robot_status, current_posture, buff_cool_value, buff_vulnerability_pct 输入; 移除 is_weak |

### 6.7 目标/巡逻节点

| XML 使用的节点 | TreeNodesModel 定义 | 关键变更 |
|---|---|---|
| `SelectObjective` | ✅ | ~30+ 输入, 新增 ammo_allow, ammo_target, field_*, fortress_ammo, 8 个坐标 (均使用 supply_zone_x/y) |
| `HoldObjective` | ✅ | |
| `WaypointPatrol` | ✅ | |
| `SelectNearestDispelCard` | ✅ | 使用 supply_zone_x/y (非 supply_x/y) |
| `SelectNearestResupplyStation` | ✅ | 使用 supply_zone_x/y |
| `SelectSafeRetreatGoal` | ✅ | 使用 supply_zone_x/y |

### 6.8 后勤节点

| XML 使用的节点 | TreeNodesModel 定义 | 状态 |
|---|---|---|
| `HoldAndHeal` | ✅ | |
| `HoldForSupplyAmmoTick` | ✅ | |

### 6.9 条件节点

| XML 使用的节点 | TreeNodesModel 定义 | 关键变更 |
|---|---|---|
| `RmucIsGameTime` | ✅ | |
| `RmucIsDead` | ✅ | |
| `IsWeakness` | ✅ | 直接读取 robot_status (非 is_weak 黑板 key) |
| `RmucIsHPBelow` | ✅ | |
| `IsAmmoBelow` | ✅ | |
| `IsCriticalState` | ✅ | |
| `IsBaseThreatened` | ✅ | 新增 outpost_alive 输入 |
| `HasValidTarget` | ✅ | |
| `IsCombatAllowed` | ✅ | 使用 robot_status (非 is_weak) |
| `IsFireWindowOk` | ✅ | 新增 robot_status, current_posture, buff_cool_value, buff_vulnerability_pct; 移除 is_weak |
| `IsZoneCardDetected` | ✅ | |
| `ShouldChassisSpin` | ✅ | NEW — 检查 distance/posture/power_boost |
| `IsAnyDispelCardDetected` | ✅ | |
| `IsAtGoal` | ✅ | |
| `RmucIsAtNavGoal` | ✅ | NEW |
| `RmucIsSupplyCardDetected` | ✅ | NEW |

### 6.10 内置节点（无需 TreeNodesModel 定义）

| 节点 | 类型 |
|---|---|
| `SetBlackboard` | 内置 Action |
| `WhileDoElse` | 内置 Control |
| `RateController` | 内置 Decorator |
| `Sequence` / `ReactiveSequence` / `Fallback` / `ReactiveFallback` | 内置 Control |
| `SubTree` | 内置 |

### 6.11 独立文件 TreeNodesModel

以下文件添加了临时 TreeNodesModel 用于 Groot2 独立加载：

| 文件 | 包含的节点定义 | 状态 |
|---|---|---|
| `CommandHub.xml` | DecidePosture, DecideEconomyCmd, DecideRespawnCmd, SentryCmdMux, RateController | ✅ |
| `EngageCombat.xml` | HasValidTarget, IsCombatAllowed, ShouldChassisSpin, RmucRobotControl, RateController | ✅ |

---

## 7. 已知限制与风险项

### 7.1 已确认的限制

| 项目 | 描述 | 影响 | 建议 |
|---|---|---|---|
| DecidePosture 强集成 | cpp 内置多层评分逻辑，XML 层无法可视化决策过程 | Groot2 只显示一个方块 | 可接受，评分+滞回模型不适合拆到 XML |
| active_subtree 一帧延迟 | CommandHub 在 ReactiveFallback 之前 tick | 几乎无影响（5s 冷却兜底） | 保持现状 |
| DeathAndRespawn.xml 废弃 | 未被 include 或引用 | 占磁盘空间，可能误导开发者 | 删除或加 DEPRECATED 标记 |
| 14 个有写无读 key | ParseSentryBlackboard 输出但无读取 | 预留字段，不影响运行 | 后续版本按需启用 |
| `is_weak` 已完全移除 | 虚弱判断统一由 IsWeakness 条件节点处理 | 旧代码引用 is_weak 会编译失败 | 确保所有 cpp 引用已清理 |

### 7.2 待确认项

| 项目 | 描述 | 需要确认 |
|---|---|---|
| 所有 TODO 坐标 | InitSentryConfig 中大量 `default="0.0"` 的坐标 | 上场前必须填写实际坐标 |
| 大能量机关激活 | `enable_big_energy` 由 DecideEconomyCmd 输出，但激活条件和流程未在 XML 中体现 | 确认仿真中是否需要额外子树 |
| search_timeout_ms 超时处理 | RespawnRecovery 搜索补给卡超时后的行为 | 确认 RmucMicroSearchSupplyCard 超时返回值 |

### 7.3 仿真前必做清单

- [ ] 填写 `InitSentryConfig` 中所有 `TODO` 坐标（地图坐标系）
- [ ] 在 Groot2 中加载主树，确认 port 连线无断裂
- [ ] 用 `ros2 topic echo` 验证 12 个订阅话题是否有数据
- [ ] 配置 Groot2 ZMQ 连接到 port 1668，观察运行时 tick 状态
- [ ] 验证 RespawnRecovery 完整流程：死亡→停车→复活→虚弱→导航→搜索→治疗→恢复
- [ ] 验证 EngageCombat 底盘旋转策略：ShouldChassisSpin 条件正确触发
- [ ] 验证 SelectBestTarget 敌方脆弱度优先级生效

---

## 8. 审计方法论总结

### 8.1 完整审计检查清单

经过多轮审计迭代，总结出以下完整检查清单供后续使用：

#### A. 控制流 (Control Flow)

- [ ] 每个 `Sequence` 是否应该是 `ReactiveSequence`（需要对上层状态变化做出响应？）
- [ ] 每个 `Fallback` 是否应该是 `ReactiveFallback`（需要被高优先级分支抢占？）
- [ ] `KeepRunning` 是否仅出现在 `ReactiveSequence` 的最后位置
- [ ] `ReactiveSequence` 中是否有有副作用的 Action（如 CancelNavGoal）
- [ ] 每个子树的门控条件 FAILURE 时，整棵子树是否能正确返回 FAILURE
- [ ] `RateController` 内是否只有 1 个子节点
- [ ] `Fallback` 容器中各分支是否能正确 FAILURE 以传递到下一分支

#### B. 数据流 (Data Flow)

- [ ] **每个 InputPort 的 `{key}` 是否有对应的 OutputPort/SetBlackboard 写入**
- [ ] **每个 OutputPort 的 `{key}` 是否有对应的 InputPort 读取**（有写无读不算 bug，但应记录）
- [ ] `inout_port` 是否在正确的生命周期内使用（避免竞争写入）
- [ ] `SetBlackboard` 的 `output_key` 和 `value` 是否拼写正确
- [ ] 跨子树共享的 blackboard key 是否通过 `_autoremap="true"` 传递
- [ ] 配置类 key（cfg.*）是否在 `InitSentryConfig` 中定义
- [ ] 所有命名是否已迁移到新标准（supply_zone_x/y 而非 supply_x/y）
- [ ] `is_weak` 黑板 key 是否已完全移除，改用 IsWeakness 条件节点

#### C. 时序 (Timing)

- [ ] 在同一个 `ReactiveSequence` 中，写入节点和读取节点的 tick 顺序是否正确
- [ ] 导航目标 `{nav.goal_x/y}` 是否存在多写入方竞争
- [ ] `RateController` 频率是否合理（太高浪费 CPU，太低响应迟钝）

#### D. TreeNodesModel 一致性

- [ ] 每个自定义节点在 `<TreeNodesModel>` 中都有定义
- [ ] port 名称、方向、default 值与 cpp `providedPorts()` 一致
- [ ] 不存在多余的 port（已删除的 port 要从 model 中同步移除）
- [ ] 独立文件的 TreeNodesModel（CommandHub, EngageCombat）与主 model 一致

#### E. 工程管理

- [ ] 所有 XML 通过 `xmllint --noout` 验证
- [ ] `<include>` 列表覆盖所有子树文件（当前 12 个）
- [ ] 无废弃/未引用的文件（DeathAndRespawn.xml 待处理）
- [ ] 编译通过 (`colcon build --packages-select rm_behavior_tree`)

---

## 附录 A: xmllint 批量验证命令

```bash
for f in src/RM_Behavior_Tree/rm_behavior_tree/config/rmuc_2026/*.xml; do
  xmllint --noout "$f" && echo "✅ $(basename $f)" || echo "❌ $(basename $f)"
done
```

## 附录 B: 快速查找悬空 InputPort 的 grep 命令

```bash
# 提取所有 XML 中引用的 blackboard key
grep -oP '\{[a-z_\.]+\}' src/RM_Behavior_Tree/rm_behavior_tree/config/rmuc_2026/*.xml | \
  sed 's/.*://' | sort -u

# 提取所有 output_port / inout_port 的 key
grep -oP '(output_port|inout_port).*default="\{([^"]+)\}"' \
  src/RM_Behavior_Tree/rm_behavior_tree/config/rmuc_2026/rmuc_2026.xml | \
  grep -oP '\{[^}]+\}' | sort -u

# 对比两组 key，找出只读不写的
```

## 附录 C: 优先级分支与 active_subtree 映射

| 优先级 | SubTree | active_subtree 值 | 门控条件 |
|--------|---------|-------------------|---------|
| [0] | RespawnRecovery | `"RespawnRecovery"` | RmucIsDead / IsWeakness |
| [0.5] | WeaknessRecovery | `"WeaknessRecovery"` | IsWeakness |
| [1] | CriticalSurvival | `"CriticalSurvival"` | IsCriticalState |
| [2] | BaseDefense | `"BaseDefense"` | IsBaseThreatened (含 outpost_alive) |
| [3] | EngageCombat | `"EngageCombat"` | HasValidTarget + IsCombatAllowed |
| [4] | SustainAndEconomy | `"SustainAndEconomy"` | RmucIsHPBelow / IsAmmoBelow |
| [5] | ObjectivePlanner | `"ObjectivePlanner"` | 无门控（永远执行） |
| [6] | PatrolAndScan | `"PatrolAndScan"` | 无门控（兜底） |
