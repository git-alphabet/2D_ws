# 🌳 RMUC 2026 哨兵行为树 —— 可视化流程图

> **格式说明**：`→` 表示顺序执行，`├/└` 表示分支选择，`↻` 表示每帧重复检查（ReactiveSequence）  
> **日期**：2026-04-04 | **BT.CPP v4**

---

## 一、总览：主树 `rmuc_2026`

```
                          ╔══════════════════════╗
                          ║    rmuc_2026 启动     ║
                          ╚══════════╤═══════════╝
                                     │
                    ┌────────────────↻───────────────┐
                    │     ReactiveSequence (每帧)     │
                    └──┬─────────────────────────────┘
                       │
            ┌──────────┼──────────────────┐
            ▼          ▼                  ▼
    ①感知与黑板   ②初始化(仅一次)    ③比赛阶段判断
   (12话题订阅     (配置+命令状态)     WhileDoElse
    +集中解析)                            │
                                ┌────────┴────────┐
                                ▼                  ▼
                        progress == 4?          非比赛阶段
                           比赛中                   │
                            │               ┌──────┴──────┐
                            │               │  回家待命     │
                            │               │  停止扫描     │
                            │               │  姿态=防御    │
                            │               └─────────────┘
                            ▼
               ┌────────────↻─────────────┐
               │  ReactiveSequence (比赛)  │
               └──┬───────────────────────┘
                  │
          ┌───────┴───────┐
          ▼               ▼
      CommandHub    ReactiveFallback
     (姿态/经济/     (8个优先级分支)
      复活决策)           │
       5Hz发送            │
                         ▼
```

---

## 二、优先级分支总览（ReactiveFallback）

```
ReactiveFallback ── 从上到下依次尝试，第一个返回 SUCCESS 的分支接管
    │
    ├── [P0]   RespawnRecovery     死亡停车 / 虚弱→导航补给区→刷卡回血
    │
    ├── [P0.5] WeaknessRecovery    虚弱安全网→导航最近增益点解除虚弱
    │
    ├── [P1]   CriticalSurvival    血量/热量危急→撤退到安全点
    │
    ├── [P2]   BaseDefense         基地受威胁→回防+战斗
    │
    ├── [P3]   EngageCombat        有目标+允许战斗→旋转/开火
    │
    ├── [P4]   SustainAndEconomy   低血量→回血 / 低弹药→补弹
    │
    ├── [P5]   ObjectivePlanner    控制高地/堡垒/增益点
    │
    └── [P6]   PatrolAndScan       默认巡逻扫描
```

> **注意**：每个分支被包裹在 `Sequence { SetBlackboard(active_subtree="XXX"), SubTree }` 中，  
> 用于追踪当前激活的子树名称。

---

## 三、感知层：PerceptionAndBlackboard

```
Sequence
    │
    ├─→ RmucSubGameStatus          (/game_status)         → game_status, now_ms
    ├─→ RmucSubRobotStatus         (/robot_status)        → robot_status
    ├─→ RmucSubRFIDStatus          (/rfid_status)         → rfid_status
    ├─→ RmucSubRobotPosition       (/robot_position)      → pose_x/y/yaw, is_at_nav_goal, pose
    ├─→ SubRadarTracks             (/radar/enemy_tracks)   → radar_tracks
    │
    │   ─── 以下 7 个为 P0 新增裁判系统话题 ───
    │
    ├─→ RmucSubSentryDecisionStatus (/sentry_decision_status) → 复活/姿态/兑换状态
    ├─→ RmucSubRobotBuff            (/robot_buff)             → 回血/冷却/防御/易伤/攻击加成
    ├─→ RmucSubProjectileAllowance  (/projectile_allowance)   → 堡垒存弹
    ├─→ RmucSubFieldStatus          (/field_status)           → 7个场地点占领状态
    ├─→ RmucSubEnemyMark            (/enemy_mark)             → 5个敌方易伤标记
    ├─→ RmucSubTeamPositions        (/team_positions)         → 队友位置
    ├─→ RmucSubTeamHP               (/team_hp)                → 前哨站/基地血量
    │
    └─→ ParseSentryBlackboard      (集中解析 → ~70+ 个黑板变量)
```

---

## 四、[P0] RespawnRecovery —— 死亡与虚弱恢复

```
            ┌─────────────────────────────┐
            │       RespawnRecovery        │
            │         (Fallback)           │
            └──────────┬──────────────────┘
                       │
           ┌───────────┴───────────┐
           ▼                       ▼
     分支A: 死亡?              分支B: 虚弱恢复
     (Sequence)           (ReactiveSequence)
           │                       │
    ┌──────┴──────┐         ┌──────┴──────┐
    ▼             │         ▼             │
 RmucIsDead?     │    IsWeakness?        │
    │             │    (门控:不虚弱       │
   YES            │     → FAILURE退出)    │
    │             │         │             │
    ▼             ▼        YES            │
 停车等待      导航控制         │             │
 复活读条      cmd=3           ▼             │
 (SUCCESS)    (停止导航)    RecoveryFlow     │
                           (Fallback)       │
                               │            │
                    ┌──────────┴──────┐     │
                    ▼                 ▼     │
             已检测到补给卡?     未检测到卡     │
              (Sequence)       (Sequence)    │
                    │                 │     │
                    ▼                 │     │
            RmucWaitAndHeal          │     │
            (等待回血到比例)           │     │
            heal_min_ratio           │     │
                    │                 │     │
                 SUCCESS              ▼     │
                               ┌──────────┐│
                               │导航到补给区││
                               │cmd=1     ││
                               └────┬─────┘│
                                    ▼      │
                               ┌──────────┐│
                               │ SendGoal  ││
                               │(supply)   ││
                               │ 5Hz持续   ││
                               └────┬─────┘│
                                    ▼      │
                               到达后刹车   │
                               cmd=3      │
                                    │      │
                                    ▼      │
                               初始化搜卡   │
                               计时器      │
                                    │      │
                                    ▼      │
                               ┌──────────┐│
                               │检测补给卡? ││
                               │  ├─YES→完成│
                               │  └─NO→微搜││
                               └──────────┘│
```

**状态机总结**：
```
  存活 + 不虚弱  ──→  FAILURE (正常退出，进入后续优先级)
  死亡 (hp=0)    ──→  SUCCESS (停车等待复活)
  存活 + 虚弱    ──→  SUCCESS (导航→刷卡→等回血→恢复后自然退出)
```

---

## 五、[P0.5] WeaknessRecovery —— 虚弱安全网

```
        ┌───────────────────────────────┐
        │     WeaknessRecovery           │
        │     (ReactiveSequence ↻)       │
        └──────────┬────────────────────┘
                   │
     ┌─────────────┼──────────────────────────┐
     ▼             ▼                          ▼
 IsWeakness?  SelectNearestDispelCard    SendGoal(1Hz)
 (门控)       (选最近增益点:              + 关闭开火
  │           补给区/基地buff/            + MoveAround
  │           前哨站buff)                  (到达后微调)
  │               │                          │
 虚弱=YES        写入 nav.goal_x/y           │
 不虚弱=退出      │                          │
                  └──→ 导航到增益点 ──→ 检测驱散卡 ──→ 解除虚弱
```

---

## 六、[P1] CriticalSurvival —— 危急生存

```
        ┌───────────────────────────┐
        │    CriticalSurvival        │
        │    (ReactiveSequence ↻)    │
        └──────────┬────────────────┘
                   │
     ┌─────────────┼──────────────────┐
     ▼             ▼                  ▼
IsCriticalState?  关闭扫描/旋转/开火   SelectSafeRetreatGoal
  │                                    (补给区 or 防御锚点
  │                                     中选最近的)
  ├─ hp < 80 → YES                         │
  ├─ heat > 245 → YES                      ▼
  └─ 否则 → FAILURE                    SendGoal(1Hz)
                                       "Retreat"
                                            │
                                            ▼
                                       KeepRunning
                                     (持续撤退直到
                                      条件解除)
```

> ⚠️ **不使用 CancelNavGoal**：在 ReactiveSequence 中会导致每帧取消导航，  
> 改为依赖 SendGoal 自动覆盖旧目标。

---

## 七、[P2] BaseDefense —— 基地防御

```
        ┌───────────────────────────┐
        │      BaseDefense           │
        │      (ReactiveSequence ↻)  │
        └──────────┬────────────────┘
                   │
     ┌─────────────┼──────────────────────┐
     ▼             ▼                      ▼
IsBaseThreatened? 开启扫描+开火         SendGoal(1Hz)
  │               关闭旋转              → 防御锚点
  ├─ base_threat                            │
  ├─ 基地血量 < 50%                         ▼
  ├─ 前哨站存活状态                     CombatLoop
  └─ 敌人距基地距离                     (战斗循环子树)
```

---

## 八、[P3] EngageCombat —— 交战

```
        ┌─────────────────────────────────────────┐
        │            EngageCombat                    │
        │          (ReactiveSequence ↻)              │
        └──────────┬──────────────────────────────┘
                   │
     ┌─────┬───────┼───────┬──────────┬──────────┐
     ▼     ▼       ▼       ▼          ▼          ▼
   有效   战斗    锁定     底盘旋转   CombatLoop
   目标?  允许?   当前位置  策略
     │     │      │         │
     │     │   SetBB:       │
     │     │   nav.goal =   │
     │     │   pose(当前)    │
     │     │                │
     │     │                ▼
     │     │     ┌────── ReactiveFallback ──────┐
     │     │     │                               │
     │     │     ▼                               ▼
     │     │  ShouldChassisSpin?            不旋转分支
     │     │     │                               │
     │     │  ┌──┴──┐                            │
     │     │  │条件: │                            │
     │     │  │·距目标近(arrive_radius内)          │
     │     │  │·姿态合适                          │
     │     │  │·功率增幅                          │
     │     │  └──┬──┘                            │
     │     │    YES                              │
     │     │     │                               │
     │     │     ▼                               ▼
     │     │   旋转+开火                    不旋转+开火
     │     │   chassis_spin=True            chassis_spin=False
     └─────┴─────────────────────────────────────┘

HasValidTarget: has_target=true 且 best_target 非空
IsCombatAllowed: 非虚弱 + 有弹药 + 热量未超限 + 血量 > hp_low
```

---

## 九、CombatLoop —— 战斗循环（P2/P3 共用）

```
        ┌───────────────────────────────┐
        │         CombatLoop             │
        │       (ReactiveSequence ↻)     │
        └──────────┬────────────────────┘
                   │
     ┌─────────────┼──────────┬───────────┬──────────┐
     ▼             ▼          ▼           ▼          ▼
SelectBestTarget  AimAt    射击窗口?    FireBurst   KeepRunning
  │              Target      │          (连发射击)
  │                          │
  │                   ┌──────┴──────┐
  │                   ▼             ▼
  │            IsFireWindowOk?   关闭开火
  │               │
  │            ┌──┴──┐
  │            │检查: │
  │            │·热量  │
  │            │·弹药  │
  │            │·姿态  │
  │            │·冷却  │
  │            │·易伤  │
  │            └─────┘
  │
  ├── 优先基地附近威胁目标 (<5m)
  ├── 考虑敌方易伤标记加权
  └── 选距自身最近目标
```

---

## 十、[P4] SustainAndEconomy —— 后勤补给

```
        ┌───────────────────────────┐
        │     SustainAndEconomy      │
        │     (ReactiveFallback)     │
        └──────────┬────────────────┘
                   │
           ┌───────┴───────┐
           ▼               ▼
       HealPlan        AmmoPlan
       (回血计划)       (补弹计划)


  ═══════ HealPlan 详情 ═══════

          hp < hp_low?
              │
         ─────┴─────
         ▼         ▼
      在补给区?    不在补给区
      (RFID检测)      │
         │            ▼
         ▼       SendGoal(1Hz)
    HoldAndHeal   → 补给区
    (等到hp_safe   关闭开火
     或脱战回血)    KeepRunning


  ═══════ AmmoPlan 详情 ═══════

        ammo < ammo_low?
              │
         ─────┴─────
         ▼         ▼
      在补给区?    不在补给区
      (RFID检测)      │
         │            ▼
         ▼    SelectNearestResupplyStation
  HoldForSupplyAmmoTick  (补给区/基地/前哨站)
  (等到ammo_target        │
   或下一分钟补给)         ▼
                     SendGoal(1Hz)
                     关闭开火
                     KeepRunning
```

---

## 十一、[P5] ObjectivePlanner —— 目标控制

```
        ┌────────────────────────────────┐
        │       ObjectivePlanner          │
        │      (ReactiveSequence ↻)       │
        └──────────┬─────────────────────┘
                   │
     ┌─────────────┼──────────────┬──────────────┐
     ▼             ▼              ▼              ▼
SelectObjective  SendGoal(1Hz)  到达判断      HoldObjective
     │            目标名称       (IsAtGoal      (占据等待)
     │            +坐标          arrive_radius)
     │
     ├── 输入 (~30+ 个参数)：
     │   ·位置、时间、血量、弹药
     │   ·基地血量、前哨站存活
     │   ·场地占领状态 (5个field_*)
     │   ·堡垒存弹 (fortress_ammo)
     │   ·8个候选坐标点
     │
     └── 输出：nav.goal_x/y + objective_name
              (如 "中央高地"、"梯形高地"、"堡垒"、"补给区" 等)
```

---

## 十二、[P6] PatrolAndScan —— 巡逻扫描

```
        ┌───────────────────────────┐
        │      PatrolAndScan         │
        │    (ReactiveSequence ↻)    │
        └──────────┬────────────────┘
                   │
     ┌─────────────┼──────────────┬──────────┐
     ▼             ▼              ▼          ▼
  关闭旋转     WaypointPatrol  SendGoal   KeepRunning
  关闭开火     (3个巡逻点循环   (1Hz)
  开启扫描      选最近未访问)
```

---

## 十三、CommandHub —— 指令中心

```
        ┌────────────────────────────────┐
        │         CommandHub              │
        │          (Sequence)             │
        └──────────┬─────────────────────┘
                   │
     ┌─────────────┼───────────────┬────────────────┐
     ▼             ▼               ▼                ▼
DecidePosture  DecideEconomyCmd  DecideRespawnCmd  SentryCmdMux
  (姿态决策)     (经济决策)        (复活决策)        (5Hz发送)
     │              │                │               │
     ├→ posture     ├→ allow_ammo    ├→ confirm_     └→ 复用所有输出
     │              ├→ trigger_ammo     respawn          发送 0x0120
     │              ├→ trigger_hp    ├→ confirm_
     │              ├→ enable_energy    instant
                    └→ 考虑:         └→ 考虑:
                       ·复活费用        ·免费复活可用?
                       ·累计次数        ·付费复活可用?
                       ·堡垒存弹        ·费用+累计
                       ·基地血量        ·基地血量
                       ·遥控次数        ·比赛时间
```

---

## 十四、关键数据流

```
  ┌──────────────┐     12个ROS2话题      ┌──────────────────┐
  │  裁判系统     │ ───────────────────→  │  Subscription     │
  │  雷达         │                       │  Nodes (12个)     │
  │  导航         │                       └────────┬─────────┘
  └──────────────┘                                 │
                                          原始消息存入黑板
                                                   │
                                                   ▼
                                     ┌──────────────────────┐
                                     │ ParseSentryBlackboard │
                                     │    (集中解析器)        │
                                     │    ~70+ 个输出变量     │
                                     └────────┬─────────────┘
                                              │
                         ┌────────────────────┼────────────────────┐
                         ▼                    ▼                    ▼
                  ┌──────────────┐   ┌──────────────┐   ┌──────────────┐
                  │ Condition节点 │   │  Decision节点  │   │  Action节点   │
                  │ (读取黑板)    │   │ (读取→输出)    │   │ (读取→执行)   │
                  └──────────────┘   └──────────────┘   └──────────────┘
                         │                    │                    │
                         ▼                    ▼                    ▼
                   优先级分支门控        CommandHub决策        导航/战斗/控制
```

---

## 十五、配置参数速查（InitSentryConfig）

| 参数 | 默认值 | 说明 |
|:---|:---|:---|
| `hp_critical` | 80 | 危急血量阈值 |
| `hp_low` | 180 | 低血量阈值 |
| `hp_safe` | 280 | 安全血量阈值 |
| `heat_high` | 210 | 高热量阈值 |
| `heat_critical` | 245 | 危急热量阈值 |
| `ammo_low` | 80 | 低弹药阈值 |
| `ammo_target` | 300 | 目标弹药量 |
| `arrive_radius` | 0.35m | 到达判定半径 |
| `objective_hold_ms` | 12000 | 目标点占据时间 |
| `combat_fire_burst_ms` | 180 | 连发开火时长 |
| `combat_fire_pause_ms` | 120 | 开火间隔时长 |
| `heal_wait_ms` | 3000 | 回血等待超时 |
| `heal_min_ratio` | 0.6 | 最低回血比例 |
| `search_timeout_ms` | 5000 | 搜卡超时时间 |

---

## 十六、子树文件清单

| 文件 | 子树ID | 优先级 | 说明 |
|:---|:---|:---|:---|
| `rmuc_2026.xml` | rmuc_2026 (主树) | — | 入口 + TreeNodesModel |
| `PerceptionAndBlackboard.xml` | PerceptionAndBlackboard | — | 12话题 + 集中解析 |
| `InitOnce.xml` | InitOnce | — | 配置初始化 |
| `CommandHub.xml` | CommandHub | — | 姿态/经济/复活决策 |
| `RespawnRecovery.xml` | RespawnRecovery | P0 | 死亡+虚弱恢复 |
| `WeaknessRecovery.xml` | WeaknessRecovery | P0.5 | 虚弱安全网 |
| `CriticalSurvival.xml` | CriticalSurvival | P1 | 危急生存 |
| `BaseDefense.xml` | BaseDefense | P2 | 基地防御 |
| `EngageCombat.xml` | EngageCombat | P3 | 交战 |
| `CombatLoop.xml` | CombatLoop | — | 战斗循环(P2/P3共用) |
| `SustainAndEconomy.xml` | SustainAndEconomy | P4 | 后勤入口 |
| `HealPlan.xml` | HealPlan | P4.1 | 回血计划 |
| `AmmoPlan.xml` | AmmoPlan | P4.2 | 补弹计划 |
| `ObjectivePlanner.xml` | ObjectivePlanner | P5 | 目标控制 |
| `PatrolAndScan.xml` | PatrolAndScan | P6 | 巡逻扫描 |
| ~~`DeathAndRespawn.xml`~~ | ~~DeathAndRespawn~~ | ❌ | 孤儿文件，未被引用 |
