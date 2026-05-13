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
    ├─→ RmucSubRobotPosition       (/robot_position)      → pose_x/y, is_at_nav_goal
    ├─→ SubRadarTracks             (/radar/enemy_tracks)   → radar_tracks
    │
    │   ─── 以下 P0 裁判系统话题为历史规划/归档链路，当前 live 树以 topic_map_rmuc.md 为准 ───
    │
    ├─→ RmucSubSentryDecisionStatus (/sentry_decision_status) → 复活/姿态/兑换状态（已废弃）
    ├─→ RmucSubRobotBuff            (/robot_buff)             → 回血/冷却/防御/易伤/攻击加成
    ├─→ RmucSubProjectileAllowance  (/projectile_allowance)   → 堡垒存弹
    ├─→ RmucSubFieldStatus          (/field_status)           → 7个场地点占领状态
    ├─→ RmucSubEnemyMark            (/enemy_mark)             → 5个敌方易伤标记
    ├─→ RmucSubTeamPositions        (/team_positions)         → 队友位置
    │
    └─→ ParseSentryBlackboard      (集中解析 → 前哨站/基地血量来自 robot_status)
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
  │           前哨站buff)                  (到达后用 pose_x/y 微调)
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
```

### 13.1 DecidePosture —— 姿态决策（综合评分系统）

> **模型**: 三姿态并行评分，最高分胜出（平局优先: 防御 > 移动 > 攻击）  
> **输出**: `posture_out` → 1=攻击 / 2=防御 / 3=移动

```
                                    DecidePosture
                                   (综合评分系统)
                                         |
                ─────────────────────────┼──────────────────────────
                |                        |                          |
         ┌──攻击姿态(1)──┐       ┌──防御姿态(2)──┐          ┌──移动姿态(3)──┐
         │   基础分：5    │       │   基础分：8    │          │   基础分：10   │
         │               │       │               │          │               │
         │ 因素1：35     │       │ 因素1：50     │          │ 因素1：50     │
         │ (有目标)      │       │ (易伤标记)    │          │ (无弹药)      │
         │ 因素2：20     │       │ 因素2：30     │          │ 因素2：20     │
         │ (血量>50%)    │       │ (基地受威胁)  │          │ (脱战状态)    │
         │ 因素3：8      │       │ 因素3：35     │          │ 因素3：10     │
         │ (血量30~50%) │       │ (血量<30%)    │          │ (无目标)      │
         │ 因素4：15     │       │ 因素4：15     │          │ 因素4：5      │
         │ (防御增益>0)  │       │ (血量30~50%) │          │ (血量30~50%) │
         │ 因素5：12     │       │ 因素5：25     │          │               │
         │ (冷却增益>0)  │       │ (热量超限)    │          │               │
         │ 因素6：18     │       │ 因素6：8      │          │               │
         │ (比赛>180s)  │       │ (无目标)      │          │               │
         │               │       │               │          │               │
         │ 事件：弹药≤0  │       │               │          │               │
         │ → 总分归零!   │       │               │          │               │
         │               │       │               │          │               │
         │   最高分→攻击  │       │  最高分→防御   │          │  最高分→移动   │
         └───────────────┘       └───────────────┘          └───────────────┘
```

**评分权重表：**

| 评分因素 | 攻击(1) | 防御(2) | 移动(3) | 数据来源 |
|:---|:---:|:---:|:---:|:---|
| 基础分 | 5 | 8 | 10 | — |
| 有目标 `has_target` | +35 | — | — | `{target.has}` |
| 无目标 `!has_target` | — | +8 | +10 | `{target.has}` |
| 血量 > 50% | +20 | — | — | `{hp.cur}/{hp.max}` |
| 血量 30%~50% | +8 | +15 | +5 | `{hp.cur}/{hp.max}` |
| 血量 < 30% | — | +35 | — | `{hp.cur}/{hp.max}` |
| 防御增益 > 0 | +15 | — | — | `{buff.defense_pct}` |
| 冷却增益 > 0 | +12 | — | — | `{buff.cool_value}` |
| 易伤标记 > 0 | — | **+50** | — | `{buff.vulnerability_pct}` |
| 基地受威胁 | — | +30 | — | `{base.threat}` |
| 热量超限 | — | +25 | — | `{shooter.heat}` > `{shooter.heat_limit}` |
| 比赛 > 180s | +18 | — | — | `{game.elapsed_time}` |
| 脱战状态 | — | — | +20 | `{state.disengaged}` |
| 弹药 ≤ 0 | **×0(归零)** | — | **+50** | `{ammo.allow}` |

> **平局处理**: 分数相同时，防御 > 移动 > 攻击（保守策略，优先保命）

**典型场景推演：**

| 场景 | 攻击分 | 防御分 | 移动分 | 结果 |
|:---|:---:|:---:|:---:|:---|
| 有目标+血量80%+无buff | 5+35+20=**60** | 8=**8** | 10=**10** | ✅ 攻击 |
| 有目标+血量20% | 5+35=**40** | 8+35=**43** | 10=**10** | ✅ 防御 |
| 易伤标记+有目标 | 5+35=**40** | 8+50=**58** | 10=**10** | ✅ 防御 |
| 弹药=0 | **0** | 8=**8** | 10+50=**60** | ✅ 移动 |
| 有目标+防御buff+冷却buff | 5+35+20+15+12=**87** | 8=**8** | 10=**10** | ✅ 攻击(激进) |
| 无目标+脱战 | 5=**5** | 8+8=**16** | 10+20+10=**40** | ✅ 移动 |
| 后半场+无目标 | 5+18=**23** | 8+8=**16** | 10+10=**20** | ✅ 攻击(争配额) |
| 基地受威胁+热量高+有目标 | 5+35+20=**60** | 8+30+25=**63** | 10=**10** | ✅ 防御 |

**输入端口一览：**

| 端口 | 类型 | 黑板来源 | 含义 |
|:---|:---|:---|:---|
| `hp_cur` | int | `{hp.cur}` | 当前血量 |
| `hp_max` | int | `{hp.max}` | 最大血量 |
| `heat_cur` | int | `{shooter.heat}` | 当前热量 |
| `heat_high` | int | `{shooter.heat_limit}` | 热量上限 |
| `has_target` | bool | `{target.has}` | 是否有目标 |
| `base_threat` | bool | `{base.threat}` | 基地是否受威胁 |
| `is_disengaged` | bool | `{state.disengaged}` | 是否脱战 |
| `stage_elapsed_time` | int | `{game.elapsed_time}` | 比赛已过秒数 |
| `current_posture` | int | `{sentry.current_posture}` | 裁判系统当前姿态（预留防抖） |
| `buff_cool_value` | int | `{buff.cool_value}` | 冷却增益值 |
| `buff_defense_pct` | int | `{buff.defense_pct}` | 防御增益百分比 |
| `buff_vulnerability_pct` | int | `{buff.vulnerability_pct}` | 易伤标记百分比 |
| `ammo_allow` | int | `{ammo.allow}` | 裁判系统允许发弹量 |

---

### 13.2 DecideEconomyCmd —— 经济决策

> **输出**: 4 个独立决策（互不互斥，可同时触发）  
> **特点**: 不是 if-else 链，而是 4 个独立判断

```
                                  DecideEconomyCmd
                                        |
                ────────────────────────┼──────────────────────────
                |                |               |                 |
         ┌──远程回血──┐   ┌──远程补弹──┐   ┌──弹丸配额──┐   ┌──大能量机关──┐
         │            │   │            │   │            │   │             │
         │ 脱战：是    │   │ 可远程     │   │ 弹药<目标   │   │ 基地受威胁   │
         │ 可远程     │   │ 补弹：是    │   │ 金币≥100   │   │ ：是         │
         │ 回血：是    │   │ 弹药<80    │   │            │   │ 剩余时间     │
         │ 血量<50%   │   │ (ammo_low) │   │ 每次请求    │   │ 120s~300s   │
         │            │   │            │   │ 50发       │   │             │
         │ → trig=1   │   │ → trig=1   │   │ → 累加50   │   │ → enable=1  │
         └────────────┘   └────────────┘   └────────────┘   └─────────────┘
```

**各决策详解：**

| 决策 | 输出端口 | 触发条件 | 说明 |
|:---|:---|:---|:---|
| 远程回血 | `trigger_remote_hp` | `is_disengaged=是` 且 `can_remote_heal=是` 且 `hp < hp_max/2` | 脱战+可远程+低血量 |
| 远程补弹 | `trigger_remote_ammo` | `can_remote_ammo=是` 且 `ammo_allow < ammo_low(80)` | 弹药不足时请求补弹 |
| 弹丸配额 | `allow_ammo_target_out` | `ammo_allow < ammo_target(300)` 且 `team_coins ≥ 100` | 每 tick 累加 50 发 |
| 大能量机关 | `enable_big_energy` | `base_threat=是` 且 `120 < remain_s < 300` | 比赛中期+基地受威胁 |

**输入端口一览：**

| 端口 | 类型 | 黑板来源 | 含义 |
|:---|:---|:---|:---|
| `hp_cur` | int | `{hp.cur}` | 当前血量 |
| `hp_max` | int | `{hp.max}` | 最大血量 |
| `ammo_allow` | int | `{ammo.allow}` | 当前允许发弹量 |
| `ammo_target` | int | `{cfg.ammo_target}` | 目标弹量配额 |
| `ammo_low` | int | `{cfg.ammo_low}` | 低弹量阈值 |
| `is_disengaged` | bool | `{state.disengaged}` | 是否脱战 |
| `can_remote_heal` | bool | `{economy.can_remote_heal}` | 是否可远程回血 |
| `can_remote_ammo` | bool | `{economy.can_remote_ammo}` | 是否可远程补弹 |
| `team_coins` | int | `{economy.team_coins}` | 队伍金币数 |
| `stage_remain_time` | int | `{game.remain_time}` | 比赛剩余秒数 |
| `base_threat` | bool | `{base.threat}` | 基地是否受威胁 |
| `allow_ammo_target_in` | int | `{cmd.allow_ammo_target}` | 上一轮累计配额 |

---

### 13.3 DecideRespawnCmd —— 复活决策

> **输出**: 普通复活确认 + 立即复活确认  
> **前提**: 仅在 `is_dead = 是` 时才有效

```
                                 DecideRespawnCmd
                                       |
                                  is_dead = ?
                                       |
                          ─────────────┼─────────────
                          |                          |
                     is_dead=是                 is_dead=否
                          |                          |
                  confirm_respawn=1          confirm_respawn=0
                          |                 confirm_instant=0
                          |                    (不做任何事)
                ──────────┼──────────
                |                    |
         ┌──立即复活──┐       ┌──普通复活──┐
         │            │       │            │
         │ 条件A:     │       │ 以上条件   │
         │  基地受威胁 │       │ 均不满足   │
         │  ：是       │       │            │
         │  金币≥300  │       │ → instant=0│
         │  → inst=1  │       │   (等普通)  │
         │            │       │            │
         │ 条件B:     │       └────────────┘
         │  剩余时间   │
         │  <60s      │
         │  → inst=1  │
         │            │
         └────────────┘
```

**判定逻辑：**

| 状态 | confirm_respawn | confirm_instant_respawn | 条件 |
|:---|:---:|:---:|:---|
| 未死亡 | 0 | 0 | `is_dead = 否` |
| 死亡 - 普通复活 | 1 | 0 | `is_dead = 是`，但不满足立即复活条件 |
| 死亡 - 立即复活A | 1 | 1 | `is_dead = 是` 且 `base_threat = 是` 且 `team_coins ≥ 300` |
| 死亡 - 立即复活B | 1 | 1 | `is_dead = 是` 且 `stage_remain_time < 60` |

**输入端口一览：**

| 端口 | 类型 | 黑板来源 | 含义 |
|:---|:---|:---|:---|
| `is_dead` | bool | `{state.is_dead}` | 是否已死亡 |
| `team_coins` | int | `{economy.team_coins}` | 队伍金币数 |
| `stage_remain_time` | int | `{game.remain_time}` | 比赛剩余秒数 |
| `base_hp_cur` | int | `{base.hp_cur}` | 基地当前血量 |
| `base_hp_max` | int | `{base.hp_max}` | 基地最大血量 |
| `base_threat` | bool | `{base.threat}` | 基地是否受威胁 |

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
