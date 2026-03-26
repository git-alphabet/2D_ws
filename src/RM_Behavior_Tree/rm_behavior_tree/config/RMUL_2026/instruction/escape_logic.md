# 脱困逻辑文档

> 最后更新：2026-03-27
> 版本：v4 — 局部脱困三分支（增加接近点分支）、全局排除改为路径距离3m

---

## 总体架构：双层脱困 + 增益点排除

```
                    ┌─────────────────────────────────┐
                    │        第一层：全局脱困              │
                    │   rmul_2026.xml (ForceSuccess)      │
                    │   排除：距控制区/补给区路径距离 < 3m │
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
- 距增益点（控制区/补给区）路径距离 < 3.0m 时全局脱困被**抑制**，由局部脱困全权处理
- **第二层**在各自的导航 Fallback 里，局部精确脱困

---

## 第一层：全局脱困

**文件**：`rmul_2026.xml`  
**位置**：最外层 `ReactiveSequence → ForceSuccess` 中

### 触发条件

8 秒内移动 < 0.15m **且**：
- 未检测到敌人（排除1）
- 距控制区路径距离 ≥ 3.0m（排除2，避免与局部控制区脱困竞争）
- 距补给区路径距离 ≥ 3.0m（排除3，避免与局部补给区脱困竞争）

### XML 结构

```xml
<ForceSuccess>
    <ReactiveSequence>
        <Inverter><IsDetectEnemy message="{robot_status}"/></Inverter>
        <Inverter>
            <IsWithinScope goal=控制区 arrive_radius="3.0" use_path_distance="true"/>
        </Inverter>
        <Inverter>
            <IsWithinScope goal=补给区 arrive_radius="3.0" use_path_distance="true"/>
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
| 排除半径（路径距离） | **3.0m** | `use_path_distance="true"`，与 FindEscapePoint 最大搜索半径 3.0m 对齐 |

### FindEscapePoint 无目标行为

- 后退方向 = (0, 0)，无方向偏好
- 三阶段候选点全部按 **clearance 降序**排列（最开阔位置优先）

### 为什么排除半径改为路径距离 3.0m？

之前用直线 4.5m（加 1.5m 余量）是保守做法。现在改用 `use_path_distance="true"` 直接以实际路径距离 3.0m 排除，与 `FindEscapePoint` 最大搜索半径完全对齐，消除了因障碍物导致直线短但路径长的误触发情况。

---

## 第二层 场景①：高血量巡逻脱困

**文件**：`HighHP_combat_logic.xml`  
**位置**：高血量战斗逻辑 → 控制区巡逻导航的 Fallback

### XML 结构

```xml
<ReactiveFallback name="NavUntilArrived_ControlZone">
    <!-- 到达判定：IsWithinScope 成功则跳出导航循环 -->
    <IsWithinScope goal=控制区 arrive_radius="{cfg.arrive_radius}"/>

    <ReactiveSequence>
        <RateController hz="1">
            <Fallback>
                <!-- 1) 未卡住 + 目标可通行 + Nav2未拒绝 → 正常前往控制区 -->
                <Sequence>
                    <Inverter><IsNavigationStuck goal=控制区 stuck_timeout_ms="5000" .../></Inverter>
                    <IsGoalAreaClear/>
                    <Inverter><IsNavGoalRejected latch_timeout_ms="10000"/></Inverter>
                    <SendGoal(Normal)/>
                </Sequence>
                <!-- 2) 未卡住 + 路径距离 > 3.0m + 目标被阻 → FindApproachPoint 接近点 -->
                <Sequence>
                    <Inverter><IsNavigationStuck goal=控制区 .../></Inverter>
                    <Inverter>
                        <IsWithinScope goal=控制区 arrive_radius="3.0" use_path_distance="true"/>
                    </Inverter>
                    <FindApproachPoint goal=控制区 search_radius="3.0" cost_threshold="150"/>
                    <SendGoal(Approach)/>
                </Sequence>
                <!-- 3) 兜底：卡住 / 近距被阻 / Nav2拒绝 → 脱困点 -->
                <Sequence>
                    <FindEscapePoint goal=控制区 → escape.goal_pose/>
                    <SendGoal(Escape)/>
                </Sequence>
            </Fallback>
        </RateController>
        <KeepRunning/>
    </ReactiveSequence>
</ReactiveFallback>
```

### 关键参数

| 参数 | 值 | 说明 |
|------|----|------|
| stuck_timeout_ms | 5000 | 近距离（≤ cfg.stuck_check_radius）5 秒超时 |
| far_stuck_timeout_ms | 10000 | 远距离（> cfg.stuck_check_radius）10 秒超时 |
| stuck_check_radius | `{cfg.stuck_check_radius}` | 近/远分界（yaml配置） |
| near_goal_skip_radius | **0.4** | 距目标 < 0.4m 不判定卡住 |
| IsNavGoalRejected.latch_timeout_ms | 10000 | 被拒绝后锁存 10s 再重发 |
| FindApproachPoint.arrive_radius（触发条件） | **3.0m 路径距离** | 远距被阻才走接近点分支 |
| FindEscapePoint | 有 goal | 后退方向优先 + dist_to_goal 排序 |

---

## 第二层 场景②：非战斗占点脱困

**文件**：`rmul_2026.xml`（内层控制区导航）  
三分支结构（Normal / Approach / Escape）与场景①相同，但**外层包裹方式不同**：

| 对比项 | 场景①（HighHP_combat_logic） | 场景②（rmul_2026） |
|--------|----------------------------|-----------------|
| 到达检测包裹 | `ReactiveFallback name="NavUntilArrived_ControlZone"` + `IsWithinScope` 在最外 | 无额外包裹；由外层 `ReactiveFallback B-1` 的 `IsWithinScope` 承担到达检测 |
| RateController 位置 | NavUntilArrived 内的 ReactiveSequence 里 | 直接在 B-3 的 ReactiveSequence 里 |
| KeepRunning | 在 NavUntilArrived 内 ReactiveSequence 末尾 | 在 B-3 ReactiveSequence 末尾 |

逻辑效果等价，结构上 rmul_2026 更简洁（外层 B-1/B-2/B-3 的 ReactiveFallback 本身已保证到达即退出）。

---

## 第二层 场景③：补给区恢复脱困

**文件**：`IsDeadAndDispelDebuff.xml`  
**位置**：恢复子树 → 前往补给区的 Fallback

### XML 结构

```xml
<ReactiveFallback name="NavUntilArrived">
    <IsWithinScope goal=补给区 arrive_radius="{cfg.arrive_radius}"/>  ← 到达判定
    <IsHPIncreasing message="{robot_status}"/>                       ← HP回升即退出
    <ReactiveSequence>
        <RateController hz="1">
            <Fallback>
                <!-- 1) 未卡住 + 目标区域安全 → 正常前往补给区 -->
                <Sequence>
                    <Inverter>
                        <IsNavigationStuck goal=补给区
                                           stuck_timeout_ms="5000"
                                           far_stuck_timeout_ms="10000"
                                           near_goal_skip_radius="0.0"/>  ← 不跳过！
                    </Inverter>
                    <IsGoalAreaClear check_radius="0.3"/>  ← 补给区用更小半径
                    <SendGoal(Normal)/>
                </Sequence>
                <!-- 2) 脱困：ForceSuccess 包裹清代价地图，避免清图失败终止脱困 -->
                <Sequence>
                    <ForceSuccess><ClearCostmap(local)/></ForceSuccess>
                    <ForceSuccess><ClearCostmap(global)/></ForceSuccess>
                    <FindEscapePoint goal=补给区 escape_timeout_ms="3000"/>
                    <SendGoal(Escape)/>
                </Sequence>
                <!-- 3) 脱困超时 → 强制返航 -->
                <SendGoal(Forced) goal=补给区/>
            </Fallback>
        </RateController>
        <KeepRunning/>
    </ReactiveSequence>
</ReactiveFallback>
```

### 关键差异

| 参数 | 值 | 与控制区的区别 |
|------|----|----------------|
| near_goal_skip_radius | **0.0** | 即使离补给区很近也检测卡住 |
| far_stuck_timeout_ms | 10000 | 远距离超时（同控制区） |
| IsGoalAreaClear.check_radius | **0.3** | 控制区用 1.0，补给区用更小值避免误判 |
| ClearCostmap 包裹 | ForceSuccess | 清图失败不终止脱困流程 |
| escape_timeout_ms | **3000** | 到达逃脱点后最多等 3 秒就超时 |
| Fallback 第三分支 | SendGoal(Forced) | FindEscapePoint FAILURE → 强制返航 |

### HP 回升自动退出机制

补给区导航的最外层是 `ReactiveFallback name="NavUntilArrived"`（已在上面 XML 结构中体现），每 tick 从头检查三个子节点，任意一个 SUCCESS 即退出：

| 退出条件 | 节点 | 说明 |
|----------|------|------|
| 几何距离到达 | `IsWithinScope` | 到补给区 arrive_radius 内 |
| HP 正在回升 | `IsHPIncreasing` | 脱困点恰好在补给区内，已开始回血 |
| 导航/脱困完成 | ReactiveSequence | 正常流程结束 |

即使机器人在脱困中，只要 HP 开始回升 → `IsHPIncreasing` 返回 SUCCESS → 整个 NavUntilArrived 成功退出，进入回血等待阶段。

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
| 全局 8s | IsNavigationStuck 无goal | FindEscapePoint clearance模式 | 排除增益点路径距离 3.0m |
| 局部 5s/10s | IsNavigationStuck 有goal，双超时 | FindEscapePoint 后退方向优先 | 路径验证+黑名单 |
| 补给区 | 同上，skip=0.0 | FindEscapePoint 3s超时 | 超时→强制返航 |
| 进展超时 | FindEscapePoint 内部 8s | 黑名单+重搜 | 30s 过期 |
| 路径检查 | isPathClear 采样 | 跳过不可达候选点 | 每 0.1m |

---

## 变更历史

| 日期 | 变更内容 |
|------|----------|
| 2026-03-27 | 局部控制区脱困改为三分支：增加 FindApproachPoint 接近点分支（远距被阻时不直接脱困，而是先靠近） |
| 2026-03-27 | 全局脱困排除半径：直线4.5m → 路径距离3.0m（use_path_distance=true），精确对齐 FindEscapePoint 搜索范围 |
| 2026-03-27 | near_goal_skip_radius 修正：0.8 → 0.4（控制区）；B-3外层增加 ReactiveFallback NavUntilArrived 包裹 |
| 2026-03-27 | 正常导航分支新增 IsNavGoalRejected 条件（Nav2拒绝时直接走脱困，避免反复重发被拒目标） |
| 2026-03-26 | 全局脱困增益点排除：距控制区/补给区 < 4.5m 时不触发，避免与局部脱困目标竞争 |
| 2026-03-26 | 合并 IsRobotStuck + IsNavigationStuck → 统一脱困架构 |
| 2026-03-26 | FindEscapePoint 新增无目标模式（clearance-only 排序） |
| 2026-03-25 | 新增三项修复：路径可达性验证、进展超时+黑名单、双超时 |
| 2026-03-25 | IsNavigationStuck 新增 far_stuck_timeout_ms / near_goal_skip_radius |
