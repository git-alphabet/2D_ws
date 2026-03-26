# 脱困逻辑文档

> 最后更新：2026-03-26
> 版本：v3 — 全局脱困增益点排除，避免与局部脱困竞争

---

## 总体架构：双层脱困 + 增益点排除

```
                    ┌─────────────────────────────────┐
                    │        第一层：全局脱困           │
                    │   rmul_2026.xml (ForceSuccess)   │
                    │   排除：距控制区/补给区 < 4.5m    │
                    └──────────┬──────────────────────┘
                               ↓
                    ┌─────────────────────────────────┐
                    │      第二层：局部脱困 × 3场景     │
                    │  ① 高血量巡逻 (HighHP)          │
                    │  ② 非战斗占点 (rmul_2026)       │
                    │  ③ 补给区恢复 (IsDeadAndDispel)  │
                    └─────────────────────────────────┘
```

- **第一层**在最外层 ReactiveSequence 中，用 `ForceSuccess` 包裹，不影响主逻辑
- 距增益点（控制区/补给区）< 4.5m 时全局脱困被**抑制**，由局部脱困全权处理
- **第二层**在各自的导航 Fallback 里，局部精确脱困

---

## 第一层：全局脱困

**文件**：`rmul_2026.xml`  
**位置**：最外层 `ReactiveSequence → ForceSuccess` 中

### 触发条件

8 秒内移动 < 0.15m **且**：
- 未检测到敌人（排除1）
- 距控制区 ≥ 4.5m（排除2，避免与局部控制区脱困竞争）
- 距补给区 ≥ 4.5m（排除3，避免与局部补给区脱困竞争）

### XML 结构

```xml
<ForceSuccess>
    <ReactiveSequence>
        <Inverter><IsDetectEnemy message="{robot_status}"/></Inverter>
        <Inverter>
            <IsWithinScope goal=控制区 arrive_radius="4.5"/>
        </Inverter>
        <Inverter>
            <IsWithinScope goal=补给区 arrive_radius="4.5"/>
        </Inverter>
        <IsNavigationStuck stuck_timeout_ms="8000" (无goal)/>
        <ClearCostmap(local)/>
        <ClearCostmap(global)/>
        <FindEscapePoint (无goal → clearance-only 排序)/>
        <SendGoal(GlobalEscape)/>
    </ReactiveSequence>
</ForceSuccess>
```

### 关键参数

| 参数 | 值 | 说明 |
|------|----|------|
| stuck_timeout_ms | 8000 | 8 秒未移动判定卡住 |
| **无 goal_x/goal_y** | — | 全局模式 |
| FindEscapePoint | clearance-only | 按开阔度排序，找最安全位置 |
| 排除半径 | 4.5m | 覆盖 FindEscapePoint 最大搜索半径 3.0m + 余量 |

### FindEscapePoint 无目标行为

- 后退方向 = (0, 0)，无方向偏好
- 三阶段候选点全部按 **clearance 降序**排列（最开阔位置优先）

### 为什么 4.5m？

局部脱困的 FindEscapePoint 搜索最远 3.0m。加 1.5m 余量 = 4.5m，确保局部逃脱点覆盖范围内全局不介入。

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
            <IsNavigationStuck goal=控制区
                               stuck_timeout_ms="5000"
                               far_stuck_timeout_ms="10000"
                               near_goal_skip_radius="0.8"/>
        </Inverter>
        <IsGoalAreaClear/>
        <SendGoal(Normal)/>
    </Sequence>
    <!-- 2) 卡住 → 清代价地图 + 搜逃脱点 -->
    <Sequence>
        <ClearCostmap × 2 (ForceSuccess)/>
        <FindEscapePoint goal=控制区/>  ← 后退方向优先
        <SendGoal(Escape)/>
    </Sequence>
</Fallback>
```

### 关键参数

| 参数 | 值 | 说明 |
|------|----|------|
| stuck_timeout_ms | 5000 | 近距离（≤ 1.6m）5 秒超时 |
| far_stuck_timeout_ms | 10000 | 远距离（> 1.6m）10 秒超时 |
| stuck_check_radius | 1.6m | 近/远分界 |
| near_goal_skip_radius | 0.8 | 距目标 < 0.8m 不判定卡住 |
| FindEscapePoint | 有 goal | 后退方向优先 + dist_to_goal 排序 |

---

## 第二层 场景②：非战斗占点脱困

**文件**：`rmul_2026.xml`（内层控制区导航）  
逻辑与场景①完全相同（相同目标、相同参数）。

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
            <IsNavigationStuck goal=补给区
                               stuck_timeout_ms="5000"
                               far_stuck_timeout_ms="10000"
                               near_goal_skip_radius="0.0"/>  ← 不跳过！
        </Inverter>
        <IsGoalAreaClear/>
        <SendGoal(Normal)/>
    </Sequence>
    <!-- 2) 脱困 -->
    <Sequence>
        <ClearCostmap × 2/>
        <FindEscapePoint goal=补给区 escape_timeout_ms="3000"/>
        <SendGoal(Escape)/>
    </Sequence>
    <!-- 3) 脱困超时 → 强制返航 -->
    <SendGoal(Forced) goal=补给区/>
</Fallback>
```

### 关键差异

| 参数 | 值 | 与控制区的区别 |
|------|----|----------------|
| near_goal_skip_radius | **0.0** | 即使离补给区很近也检测卡住 |
| escape_timeout_ms | **3000** | 到达逃脱点后最多等 3 秒就超时 |
| Fallback 第三分支 | SendGoal(Forced) | FindEscapePoint FAILURE → 强制返航 |

### HP 回升自动退出机制

补给区导航的最外层是 `ReactiveFallback name="NavUntilArrived"`，每 tick 从头检查：

```xml
<ReactiveFallback name="NavUntilArrived">
    <IsWithinScope .../>                         ← 判定1：几何距离到达
    <IsHPIncreasing message="{robot_status}"/>   ← 判定2：HP在回升
    <ReactiveSequence>                           ← 导航+脱困逻辑
        ...
    </ReactiveSequence>
</ReactiveFallback>
```

即使机器人在脱困中，只要 HP 开始回升 → `IsHPIncreasing` 返回 SUCCESS → 跳出脱困进入回血阶段。

---

## IsNavigationStuck 算法

### 端口

| 端口 | 默认值 | 说明 |
|------|--------|------|
| pose_x/pose_y | — | 机器人位置（必须） |
| goal_x/goal_y | — | 目标位置（可选，不提供=全局模式） |
| stuck_timeout_ms | 5000 | 近距离超时 |
| far_stuck_timeout_ms | 10000 | 远距离超时（0=不启用） |
| stuck_check_radius | 1.6 | 近/远分界距离 |
| near_goal_skip_radius | 0.0 | 距目标 < 此值不判定卡住 |
| movement_threshold | 0.15 | 移动阈值 |
| reset_distance | 0.5 | 解除卡住锁存距离 |

### 超时选择

```
if (无 goal):
    effective_timeout = stuck_timeout_ms          # 全局模式

elif (dist < near_goal_skip_radius):
    return FAILURE                                # 已到达

elif (dist ≤ stuck_check_radius):
    effective_timeout = stuck_timeout_ms          # 近：5s

elif (far_stuck_timeout_ms > 0):
    effective_timeout = far_stuck_timeout_ms      # 远：10s

else:
    return FAILURE                                # 远且未启用
```

---

## FindEscapePoint 算法

### 端口

| 端口 | 默认值 | 说明 |
|------|--------|------|
| robot_x/robot_y | — | 机器人位置（必须） |
| goal_x/goal_y | — | 原始目标（可选，不提供=clearance模式） |
| escape_timeout_ms | 0 | 到达逃脱点后超时（0=不超时） |
| arrive_radius | 0.3 | 到达判定半径 |

### 三阶段搜索

| 阶段 | cost阈值 | 搜索半径 | 有goal排序 | 无goal排序 |
|------|----------|----------|------------|------------|
| Phase1 | <50 | 0.5~2.0m | 后退方向优先 | clearance降序 |
| Phase2 | <150 | 0.5~3.0m | 后退方向优先 | clearance降序 |
| Phase3 | <235 | 0.5~3.0m | 任意方向 | clearance降序 |

### 候选点过滤

1. **cost 阈值**：候选点代价 < 阶段阈值
2. **路径可达性** (`isPathClear`)：每 0.1m 采样，cost < 253
3. **黑名单** (`isBlacklisted`)：距黑名单点 ≥ 0.4m

### 进展超时与黑名单

- **进展超时**：已提交点 8 秒无进展（距离未减 ≥ 0.2m） → 黑名单 + 重搜
- **黑名单**：半径 0.4m，30 秒过期

---

## 安全网总结

| 防线 | 检测机制 | 逃脱方式 | 特殊处理 |
|------|----------|----------|----------|
| 全局 8s | IsNavigationStuck 无goal | FindEscapePoint clearance模式 | 排除增益点 4.5m |
| 局部 5s/10s | IsNavigationStuck 有goal，双超时 | FindEscapePoint 后退方向优先 | 路径验证+黑名单 |
| 补给区 | 同上，skip=0.0 | FindEscapePoint 3s超时 | 超时→强制返航 |
| 进展超时 | FindEscapePoint 内部 8s | 黑名单+重搜 | 30s 过期 |
| 路径检查 | isPathClear 采样 | 跳过不可达候选点 | 每 0.1m |

---

## 变更历史

| 日期 | 变更内容 |
|------|----------|
| 2026-03-26 | 全局脱困增益点排除：距控制区/补给区 < 4.5m 时不触发，避免与局部脱困目标竞争 |
| 2026-03-26 | 合并 IsRobotStuck + IsNavigationStuck → 统一脱困架构 |
| 2026-03-26 | FindEscapePoint 新增无目标模式（clearance-only 排序） |
| 2026-03-25 | 新增三项修复：路径可达性验证、进展超时+黑名单、双超时 |
| 2026-03-25 | IsNavigationStuck 新增 far_stuck_timeout_ms / near_goal_skip_radius |
