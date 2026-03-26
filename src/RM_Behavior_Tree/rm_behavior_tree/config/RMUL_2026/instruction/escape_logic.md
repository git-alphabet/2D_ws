# 脱困逻辑文档

> 最后更新：2026-03-26
> 版本：v2 — 合并 IsRobotStuck + IsNavigationStuck 为统一脱困架构

---

## 总体架构：双层脱困 + 三场景

```
                    ┌─────────────────────────────────┐
                    │        第一层：全局脱困           │
                    │   rmul_2026.xml (ForceSuccess)   │
                    │   任意位置卡住都能触发            │
                    └──────────┬──────────────────────┘
                               ↓
                    ┌─────────────────────────────────┐
                    │      第二层：局部脱困 × 3场景     │
                    │  ① 高血量巡逻 (HighHP)          │
                    │  ② 非战斗占点 (rmul_2026)       │
                    │  ③ 补给区恢复 (IsDeadAndDispel)  │
                    └─────────────────────────────────┘
```

- **第一层**在最外层 ReactiveSequence 中，用 `ForceSuccess` 包裹，不影响主逻辑流转
- **第二层**在各自的导航 Fallback 里，局部精确脱困

---

## 第一层：全局脱困

**文件**：`rmul_2026.xml`  
**位置**：最外层 `ReactiveSequence → ForceSuccess` 中，在 `ReactiveFallback` 之前

### 触发条件

8 秒内移动 < 0.15m **且**：
- 未检测到敌人（`IsDetectEnemy` = false）
- 未到达控制区（`IsWithinScope` = false）

### XML 结构

```xml
<ForceSuccess>
    <ReactiveSequence>
        <Inverter><IsDetectEnemy message="{robot_status}"/></Inverter>
        <Inverter>
            <IsWithinScope pose_x="{pose.x}" pose_y="{pose.y}"
                           goal_x="{cfg.control_zone_goal_x}"
                           goal_y="{cfg.control_zone_goal_y}"
                           arrive_radius="{cfg.arrive_radius}"/>
        </Inverter>
        <IsNavigationStuck pose_x="{pose.x}" pose_y="{pose.y}"
                           stuck_timeout_ms="8000"
                           movement_threshold="0.15"
                           reset_distance="0.5"/>
        <ClearCostmap service_name="local_costmap/clear_entirely" timeout_ms="1000"/>
        <ClearCostmap service_name="global_costmap/clear_entirely" timeout_ms="1000"/>
        <FindEscapePoint robot_x="{pose.x}" robot_y="{pose.y}"
                         escape_x="{nav.goal_x}" escape_y="{nav.goal_y}"
                         escape_pose="{nav.goal_pose}"/>
        <SendGoal name="GlobalEscape" goal_pose="{nav.goal_pose}"
                  frame_id="map" action_name="navigate_to_pose" min_interval_ms="1000"/>
    </ReactiveSequence>
</ForceSuccess>
```

### 关键特性

| 参数 | 值 | 说明 |
|------|----|------|
| stuck_timeout_ms | 8000 | 8 秒未移动判定卡住 |
| **无 goal_x/goal_y** | — | 全局模式：无距离/方向判断 |
| FindEscapePoint 模式 | clearance-only | 按开阔度排序，找最安全位置 |

### IsNavigationStuck 无目标行为

- 无 `goal_x/goal_y` → 跳过距离相关逻辑（无远近超时区分，无 near_goal_skip）
- 直接使用 `stuck_timeout_ms=8000` 作为唯一超时
- 8 秒内位移 < `movement_threshold`(0.15m) → 锁存 SUCCESS
- 移动 > `reset_distance`(0.5m) → 解锁，返回 FAILURE

### FindEscapePoint 无目标行为

- 后退方向 = (0, 0)，无方向偏好
- 三阶段候选点全部按 **clearance 降序**排列（最开阔位置优先）
- Phase1: cost<50, 半径 0.5~2.0m
- Phase2: cost<150, 半径 0.5~3.0m
- Phase3: cost<235, 半径 0.5~3.0m

---

## 第二层 场景①：高血量巡逻脱困

**文件**：`HighHP_combat_logic.xml`  
**位置**：高血量战斗逻辑 → 控制区巡逻导航的 Fallback

### XML 结构

```xml
<Fallback>
    <!-- 1) 未卡住 + 目标区域可通行 → 正常前往控制区 -->
    <Sequence>
        <Inverter>
            <IsNavigationStuck pose_x="{pose.x}" pose_y="{pose.y}"
                               goal_x="{cfg.control_zone_goal_x}"
                               goal_y="{cfg.control_zone_goal_y}"
                               stuck_check_radius="{cfg.stuck_check_radius}"
                               stuck_timeout_ms="5000"
                               far_stuck_timeout_ms="10000"
                               near_goal_skip_radius="0.8"/>
        </Inverter>
        <IsGoalAreaClear .../>
        <SendGoal name="PatrolControlZone_Normal" .../>
    </Sequence>
    <!-- 2) 卡住 → 清代价地图 + 搜逃脱点 -->
    <Sequence>
        <ClearCostmap × 2 (ForceSuccess)/>
        <FindEscapePoint robot_x="{pose.x}" robot_y="{pose.y}"
                         goal_x="{cfg.control_zone_goal_x}"
                         goal_y="{cfg.control_zone_goal_y}" .../>
        <SendGoal name="PatrolControlZone_Escape" .../>
    </Sequence>
</Fallback>
```

### 关键参数

| 参数 | 值 | 说明 |
|------|----|------|
| stuck_timeout_ms | 5000 | 近距离（≤ stuck_check_radius）5 秒超时 |
| far_stuck_timeout_ms | 10000 | 远距离（> stuck_check_radius）10 秒超时 |
| stuck_check_radius | {cfg} 默认 1.6m | 近/远分界距离 |
| near_goal_skip_radius | 0.8 | 距目标 < 0.8m 不判定卡住（已到达控制区） |
| FindEscapePoint | 有 goal | 后退方向优先 + dist_to_goal 排序 |

---

## 第二层 场景②：非战斗占点脱困

**文件**：`rmul_2026.xml`（内层控制区导航）  
**位置**：ReactiveFallback → 抢控制区分支

逻辑与场景①完全相同（相同目标、相同参数），是 rmul_2026.xml 主树内嵌的控制区导航。

---

## 第二层 场景③：补给区恢复脱困

**文件**：`IsDeadAndDispelDebuff.xml`  
**位置**：恢复子树 → 前往补给区的 Fallback

### XML 结构

```xml
<Fallback>
    <!-- 1) 正常导航 -->
    <Sequence>
        <Inverter>
            <IsNavigationStuck ... goal=补给区
                               near_goal_skip_radius="0.0"/>
        </Inverter>
        <IsGoalAreaClear .../>
        <SendGoal name="GoSupplyToHeal_Normal" .../>
    </Sequence>
    <!-- 2) 脱困 -->
    <Sequence>
        <ClearCostmap × 2 (ForceSuccess)/>
        <FindEscapePoint goal=补给区 escape_timeout_ms="3000" .../>
        <SendGoal name="GoSupplyToHeal_Escape" .../>
    </Sequence>
    <!-- 3) 脱困也超时 → 强制返航 -->
    <SendGoal name="GoSupplyToHeal_Forced" goal=补给区 .../>
</Fallback>
```

### 关键差异

| 参数 | 值 | 与控制区的区别 |
|------|----|----------------|
| near_goal_skip_radius | **0.0** | 即使离补给区很近也检测卡住（必须精确到达） |
| escape_timeout_ms | **3000** | 到达逃脱点后最多等 3 秒就超时 |
| Fallback 第三分支 | SendGoal(Forced) | FindEscapePoint FAILURE → 不顾一切强制返航 |

### HP 回升自动退出机制

补给区导航的最外层是 `ReactiveFallback name="NavUntilArrived"`，每 tick 从头检查三个分支：

```xml
<ReactiveFallback name="NavUntilArrived">
    <IsWithinScope .../>                         ← 判定1：几何距离到达
    <IsHPIncreasing message="{robot_status}"/>   ← 判定2：HP在回升
    <ReactiveSequence>                           ← 导航+脱困逻辑
        ...
    </ReactiveSequence>
</ReactiveFallback>
```

**工作原理**：
1. `IsWithinScope` → 距补给区 < arrive_radius → SUCCESS → 退出导航进入回血
2. **`IsHPIncreasing`** → HP 正在回升 → SUCCESS → 退出导航进入回血
3. 两者都不满足 → 进入 ReactiveSequence（正常导航/脱困）

**关键场景**：即使机器人被判为卡住、正在执行 FindEscapePoint 脱困，只要 HP 开始回升（说明已在补给区恢复范围内），`IsHPIncreasing` 立即返回 SUCCESS → 整个 `NavUntilArrived` 判定到达 → 跳出脱困流程，进入后续回血等待。

这防止了"人已在补给区回血但因没精确到目标点还在跑脱困"的情况。

### 补给区三级 Fallback 流程

```
[正常导航] 未卡住 → SendGoal(补给区)
    ↓ FAILURE
[脱困导航] 卡住 → ClearCostmap → FindEscapePoint(3秒超时) → SendGoal(逃脱点)
    ↓ FAILURE (3秒到达逃脱点仍在等)
[强制返航] 不顾一切 → SendGoal(补给区)
```

---

## IsNavigationStuck 算法详解

### 端口

| 端口 | 默认值 | 说明 |
|------|--------|------|
| pose_x/pose_y | — | 机器人当前位置（必须） |
| goal_x/goal_y | — | 目标位置（可选，不提供时为全局模式） |
| stuck_timeout_ms | 5000 | 近距离超时 |
| far_stuck_timeout_ms | 10000 | 远距离超时（0=不启用远距离检测） |
| stuck_check_radius | 1.6 | 近/远分界距离 |
| near_goal_skip_radius | 0.0 | 距目标 < 此值不判定卡住 |
| movement_threshold | 0.15 | 移动阈值 |
| reset_distance | 0.5 | 解除卡住锁存距离 |

### 状态机

```
┌──────────┐  moved>0.15m   ┌──────────────┐
│ 初始化   │ ──────────────→│ 正常追踪     │
│(第一tick)│                │ (FAILURE)    │←──────┐
└──────────┘                └──────┬───────┘       │
                                   │  静止>timeout  │
                                   ↓                │ moved>0.5m
                            ┌──────────────┐       │
                            │ 锁存卡住     │───────┘
                            │ (SUCCESS)    │
                            └──────────────┘
```

### 超时选择逻辑

```
if (无 goal_x/goal_y):
    effective_timeout = stuck_timeout_ms          # 全局模式

elif (dist_to_goal < near_goal_skip_radius):
    return FAILURE                                # 已到达，不算卡住

elif (dist_to_goal <= stuck_check_radius):
    effective_timeout = stuck_timeout_ms          # 近距离：5s

elif (far_stuck_timeout_ms > 0):
    effective_timeout = far_stuck_timeout_ms      # 远距离：10s

else:
    return FAILURE                                # 远距离且未启用远超时
```

---

## FindEscapePoint 算法详解

### 端口

| 端口 | 默认值 | 说明 |
|------|--------|------|
| robot_x/robot_y | — | 机器人位置（必须） |
| goal_x/goal_y | — | 原始目标（可选，不提供时为全局开阔区模式） |
| costmap_topic | global_costmap/costmap_raw | costmap 订阅 |
| escape_timeout_ms | 0 | 到达逃脱点后的超时（0=不超时） |
| arrive_radius | 0.3 | 到达判定半径 |

### 状态机

```
                    ┌──────────────┐
     2s+未tick ───→│  重置状态     │
                    └──────┬───────┘
                           ↓
                    ┌──────────────┐
                    │  超时冷却中？ │──→ <6s: return FAILURE
                    │  (timed_out) │──→ ≥6s: 重置，继续
                    └──────┬───────┘
                           ↓
                    ┌──────────────┐  cost≥235 or dist>5m
     研发新点 ←────│  已提交点有效？│──────────────────────→ 重搜
                    └──────┬───────┘
                           │ 有效
                           ↓
                    ┌──────────────┐  8s无进展
                    │  进展检测     │──────────────→ 黑名单+重搜
                    └──────┬───────┘
                           │ 正常
                           ↓
                    ┌──────────────┐  ≥escape_timeout_ms
                    │  到达逃脱点？ │──────────────→ timed_out=true
                    │  (dist<0.3m) │               return FAILURE
                    └──────┬───────┘
                           │ 未到达 or 未超时
                           ↓
                    return SUCCESS (继续使用当前已提交点)
```

### 三阶段搜索

| 阶段 | cost阈值 | 搜索半径 | 方向偏好（有goal时） | 方向偏好（无goal时） |
|------|----------|----------|---------------------|---------------------|
| Phase1 | <50 | 0.5~2.0m | 后退方向优先 | clearance降序 |
| Phase2 | <150 | 0.5~3.0m | 后退方向优先 | clearance降序 |
| Phase3 | <235 | 0.5~3.0m | 任意方向 | clearance降序 |

### 候选点过滤

每个候选点必须通过：
1. **cost 阈值**：候选点代价 < 阶段阈值
2. **路径可达性** (`isPathClear`)：机器人→候选点直线路径上每 0.1m 采样，cost < 253
3. **黑名单过滤** (`isBlacklisted`)：候选点距黑名单任意点 ≥ 0.4m

### 候选点排序

**有目标模式** (`has_goal=true`):
- Phase1/2 (`prefer_retreat=true`):
  - 首先：后退方向候选点（dot > 0）排在前面
  - 其次：`score = dist_to_goal - 0.1 × clearance`（越小越好）
- Phase3 (`prefer_retreat=false`):
  - 纯 score 排序（无方向偏好）

**无目标模式** (`has_goal=false`):
- 所有阶段纯按 **clearance 降序**（最开阔位置优先）

### 进展超时与黑名单

- **进展超时**：已提交点存在 > 8 秒且距离未减少 > 0.2m → 判定不可达
- **黑名单**：不可达点加入黑名单，半径 0.4m 内不再选取，30 秒后过期

---

## 安全网总结

| 防线 | 检测机制 | 逃脱方式 | 特殊处理 |
|------|----------|----------|----------|
| 全局 8s | IsNavigationStuck 无goal | FindEscapePoint clearance模式 | 最开阔位置 |
| 局部 5s/10s | IsNavigationStuck 有goal，双超时 | FindEscapePoint 后退方向优先 | 路径验证+黑名单 |
| 补给区 | 同上，skip=0.0 | FindEscapePoint 3s超时 | 超时后强制返航 |
| 进展超时 | FindEscapePoint 内部 8s | 黑名单该点+重搜 | 30s 过期 |
| 路径检查 | isPathClear 采样 | 跳过不可达候选点 | 每 0.1m 采样 |

---

## 变更历史

| 日期 | 变更内容 |
|------|----------|
| 2026-03-26 | 合并 IsRobotStuck + IsNavigationStuck → 统一脱困架构 |
| 2026-03-26 | FindEscapePoint 新增无目标模式（clearance-only 排序） |
| 2026-03-26 | 全局脱困升级：8s检测 → 清地图 → 搜最开阔点 → SendGoal |
| 2026-03-25 | 新增三项修复：路径可达性验证、进展超时+黑名单、双超时 |
| 2026-03-25 | IsNavigationStuck 新增 far_stuck_timeout_ms / near_goal_skip_radius |
