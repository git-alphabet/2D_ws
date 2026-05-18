# 姿态选择系统 (Posture Selection)

## 姿态分布表

| 状态 | 子树 | 姿态 | 说明 |
|------|------|------|------|
| 初始化 | InitOnce | 3 移动 | 黑板默认值 |
| 基地防御 | BaseDefense | 2 防御 | 到防御锚点后驻守 |
| 低血量/死亡 | LowHPRetreat | 3 移动 | 1.5倍移速跑路 |
| 补弹 | SustainAndEconomy | 2 防御 | 减伤50%安全补弹 |
| 检测到敌人 | EnemyHold | 不改姿态 | 只取消导航并让底盘自转 |
| 正常巡逻（导航中） | ObjectivePlanner | 3 移动 | 1.5倍移速跑图 |
| 到达目标点（无敌人） | ObjectivePlanner | 2 防御 | 驻守减伤 |

## 架构设计

### 分散式 SelectPosture

姿态选择不在 CommandHub 统一决策，而是**分散到各战术子树**中，每个子树自行设置期望姿态：

```
ObjectivePlanner.xml   → SelectPosture posture_value="3"
BaseDefense.xml        → SelectPosture posture_value="2"
LowHPRetreat.xml       → SelectPosture posture_value="3"
SustainAndEconomy.xml  → SelectPosture posture_value="2"
EnemyHold              → 不调用 SelectPosture
```

### 数据流

```
各子树 SelectPosture → {cmd.posture} → PostureDegradationGuard → {cmd.final_posture} → SentryCmdMux → 裁判系统
```

- SelectPosture 自带 5 秒冷却防抖（匹配裁判系统冷却限制）
- PostureDegradationGuard 在 CommandHub 中，作为姿态发布前的最终守卫
- SentryCmdMux 以 5Hz 发布指令

## PostureDegradationGuard（姿态降级守卫）

### 目的

防止某个姿态累计使用超过 3 分钟导致效果降级。通过"卡 timing"方式，在达到阈值时强制切换防御姿态 5 秒，重置该姿态的累计计时器。

### 工作流程

```
正常运行：状态A → 姿态a → 计时器累加
              ↓
累计 time(a) ≥ 3分钟 → 触发降级守卫
              ↓
强制切换为 防御姿态(2)，持续 5 秒
同时：重置姿态 a 的累计计时器归零
              ↓
5秒后：
  ├─ 状态没变 (A=B) → 恢复原姿态 a（计时器从 0 重新开始）
  └─ 状态变了 (A≠B) → 自动匹配新状态的姿态 b
```

### XML 配置参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `degradation_threshold_s` | 180 | 姿态累计使用阈值（秒） |
| `forced_defense_s` | 5 | 强制防御持续时间（秒，匹配裁判系统冷却） |

### 所在位置

`CommandHub.xml` 中 SentryCmdMux 前：

```xml
<PostureDegradationGuard desired_posture="{cmd.posture}"
                         degradation_threshold_s="180"
                         forced_defense_s="5"
                         final_posture="{cmd.final_posture}"/>
```

## 相关文件

- `select_posture.hpp / .cpp` — SelectPosture 节点（带 5s 冷却防抖）
- `posture_degradation_guard.hpp / .cpp` — PostureDegradationGuard 节点
- `posture_scoring.md` — 姿态评分与规则详解
