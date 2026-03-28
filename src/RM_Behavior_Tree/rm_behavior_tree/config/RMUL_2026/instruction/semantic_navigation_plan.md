# 语义导航 / 语义区域接入方案

> 最后更新：2026-03-28
> 版本：v1 — 基于现有 Nav2 + RMUL_2026 行为树的落地方案

---

## 目标

在**已知地图**前提下，为机器人增加“地图语义区域”能力，使系统能够理解：

- 哪些区域**不能走**
- 哪些区域**可以走但代价更高**
- 哪些区域进入后要**开小陀螺**
- 哪些区域进入后要**停云台扫描 / 搜敌 / 切换行为**

本方案不要求先上完整 semantic SLAM，而是在当前工程基础上做：

1. 已有地图 / 定位
2. 语义区域配置
3. Nav2 语义代价层
4. 行为树语义条件节点

这是当前仓库中**成本最低、收益最高、最容易稳定落地**的路线。

---

## 先说结论

对于当前工程，最合适的不是直接重做一套“完整语义 SLAM”，而是：

```text
已有地图 + 语义区域配置 + Nav2 语义 costmap layer + BT 语义条件节点
```

其中：

- **禁行区 / 高风险区 / 绕行区** → 进入 **Nav2 costmap**
- **进入某区域要开小陀螺 / 停车 / 搜敌 / 切 recovery** → 进入 **Behavior Tree**

不要把两类需求混在一个模块里。

---

## 为什么现在不必先做完整 semantic SLAM

“完整 semantic SLAM”通常指：

1. 建图过程中同步识别语义
2. 在线更新地图中的对象或区域标签
3. 地图不仅有几何信息，还有门、墙、危险区、控制区、补给区等语义信息

但当前需求是：

- 地图已经知道
- 区域规则基本可预先定义
- 重点是让规划器和行为树理解这些区域

因此更准确的目标其实是：

- **语义地图叠加**
- **语义导航**
- **语义行为决策**

而不是先做一整套重型 semantic SLAM。

---

## 当前工程里已经具备的接入点

### 1. Nav2 代价地图层

当前 local/global costmap 已经采用插件式结构：

- `static_layer`
- `intensity_voxel_layer`
- `inflation_layer`

可见：

- `src/gxu2026_sentry_nav/gxu2026_nav_bringup/config/simulation/nav2_params.yaml`
- `src/gxu2026_sentry_nav/gxu2026_nav_bringup/config/reality/nav2_params.yaml`

其中当前仿真参数里已经配置：

```yaml
plugins: ["static_layer", "intensity_voxel_layer", "inflation_layer"]
```

说明导航层已经天然支持再插入一层新的语义 layer。

### 2. 自定义 Costmap 插件基础已经存在

当前已有自定义层：

- `pb_nav2_costmap_2d::IntensityVoxelLayer`

源码：

- `src/gxu2026_sentry_nav/pb_nav2_plugins/src/layers/intensity_voxel_layer.cpp`

这意味着工程已经具备：

1. 自定义 Nav2 layer 的包结构
2. 插件注册与参数化方式
3. 在 launch / yaml 中启用自定义 layer 的工程惯例

所以新增语义 layer 的工程风险很低。

### 3. 行为树已经支持“区域驱动”的决策方式

当前 RMUL_2026 行为树里已经有很多典型区域逻辑：

- 控制区目标点
- 补给区目标点
- RFID 到达判断
- 基于 costmap 的接近点 / 脱困点搜索

相关文件：

- `src/RM_Behavior_Tree/rm_behavior_tree/config/RMUL_2026/rmul_2026.xml`
- `src/RM_Behavior_Tree/rm_behavior_tree/config/RMUL_2026/HighHP_combat_logic.xml`
- `src/RM_Behavior_Tree/rm_behavior_tree/config/RMUL_2026/IsDeadAndDispelDebuff.xml`

相关节点示例：

- `IsGoalAreaClear`
- `FindApproachPoint`
- `FindEscapePoint`
- `IsControlZoneDetected`
- `IsWithinScope`

说明行为树层已经接受“区域 / 位置 / costmap / RFID 事件”这种决策风格，非常适合再接一个通用语义区节点。

---

## 建议的总体架构

建议将“语义”拆成两层：

### A. 导航语义

作用：影响路径规划和控制器避障。

适合放在 Nav2 costmap 中的语义：

1. 禁行区
2. 高代价区
3. 狭窄通道
4. 风险走廊

### B. 行为语义

作用：影响行为树决策与机器人状态。

适合放在 BT 中的语义：

1. 进入某区后开启小陀螺
2. 进入某区后停云台扫描
3. 进入某区后优先搜敌
4. 进入某区后禁止追敌
5. 进入某区后切换 recovery / defend / patrol 模式

### 设计边界

- “**能不能走**”属于导航语义
- “**到了以后做什么**”属于行为语义

这两个层面不要互相替代。

---

## 最推荐的实现路线：静态语义区域

适用条件：

- 地图已知
- 区域基本固定
- 需要先尽快落地比赛可用版本

### 第一步：新增一份语义区域配置文件

建议单独维护，不要把区域硬编码在 XML 里。

建议位置：

- `src/gxu2026_sentry_nav/gxu2026_nav_bringup/config/simulation/semantic_zones.yaml`
- `src/gxu2026_sentry_nav/gxu2026_nav_bringup/config/reality/semantic_zones.yaml`

推荐结构：

```yaml
semantic_zones:
  - name: forbidden_corner
    type: keepout
    points:
      - [1.0, 2.0]
      - [2.5, 2.0]
      - [2.5, 3.2]
      - [1.0, 3.2]

  - name: control_zone
    type: spin_zone
    points:
      - [4.7, -4.3]
      - [5.5, -4.3]
      - [5.5, -3.5]
      - [4.7, -3.5]

  - name: danger_lane
    type: high_cost
    cost: 220
    points:
      - [2.0, -1.0]
      - [6.0, -1.0]
      - [6.0, 0.0]
      - [2.0, 0.0]

  - name: slow_passage
    type: slow_zone
    max_linear_vel: 0.3
    points:
      - [3.0, -2.0]
      - [4.0, -2.0]
      - [4.0, -1.0]
      - [3.0, -1.0]
```

### 为什么建议统一用 polygon

当前很多逻辑还是：

- `goal_x`
- `goal_y`
- `arrive_radius`

这适合单点目标，但不适合复杂区域。控制区、禁行区、慢行区一般都不是完美圆形。

因此建议语义区统一采用 **polygon**：

1. Nav2 layer 可以直接填充 polygon
2. BT 可以做点在多边形内判断
3. RViz marker 也容易可视化

---

## 导航侧实现：SemanticZoneLayer

### 目标

把“哪些地方不能走 / 哪些地方更危险”写进 local/global costmap。

### 放置位置

建议直接放在现有 Nav2 插件包：

- `src/gxu2026_sentry_nav/pb_nav2_plugins/`

新增文件：

- `include/pb_nav2_plugins/layers/semantic_zone_layer.hpp`
- `src/layers/semantic_zone_layer.cpp`

### 职责

`SemanticZoneLayer` 启动后做以下事情：

1. 读取 `semantic_zones.yaml`
2. 将 polygon 转换到 costmap 栅格上
3. 按 `zone type` 写入不同代价值

### 推荐映射规则

#### keepout

- 直接写 `LETHAL_OBSTACLE`
- 规划器和控制器都会统一视为不可进入

#### high_cost

- 写较高代价值，例如 180~250
- 允许经过，但只有在绕不开时才考虑

#### slow_zone

- 不建议直接映射为 cost
- 更适合给 controller 或 BT 提供语义事件

#### preferred_lane

- 当前阶段不建议实现
- Nav2 costmap 更擅长惩罚项，不擅长奖励项

### 在 Nav2 参数中启用方式

当前插件列表为：

```yaml
plugins: ["static_layer", "intensity_voxel_layer", "inflation_layer"]
```

后续建议改为：

```yaml
plugins: ["static_layer", "semantic_zone_layer", "intensity_voxel_layer", "inflation_layer"]

semantic_zone_layer:
  plugin: pb_nav2_costmap_2d::SemanticZoneLayer
  enabled: true
  semantic_map_yaml: /ws/src/.../semantic_zones.yaml
```

### 为什么“禁行区”一定要进 costmap

如果只在行为树里写“这里不能去”，会有两个问题：

1. planner 仍可能把路径规划进去
2. controller 仍可能在局部跟踪时钻进去

只有写进 costmap，整个 Nav2 栈才会统一承认这个区域“不可走”或“高风险”。

---

## 行为树侧实现：语义区条件节点

### 目标

让行为树理解：进入某区域后要执行什么动作。

### 放置位置

建议放在：

- `src/RM_Behavior_Tree/rm_behavior_tree/plugins/rmul_2026/condition/`

新增文件：

- `is_in_semantic_zone.hpp`
- `is_in_semantic_zone.cpp`

### 建议节点接口

最小版条件节点：

```xml
<IsInSemanticZone zone_name="control_zone"
                  pose_x="{pose.x}"
                  pose_y="{pose.y}"/>
```

更通用的版本也可以做成：

```xml
<GetCurrentSemanticZone pose_x="{pose.x}"
                        pose_y="{pose.y}"
                        zone_name="{semantic.zone_name}"
                        zone_type="{semantic.zone_type}"/>
```

### 典型用法

例如进入控制区后开小陀螺：

```xml
<ReactiveSequence>
  <IsInSemanticZone zone_name="control_zone"
                    pose_x="{pose.x}"
                    pose_y="{pose.y}"/>
  <RobotControl stop_gimbal_scan="False" chassis_spin="True"/>
  <NavControlCmd cmd_type="3" emergency_stop="False"/>
  <KeepRunning/>
</ReactiveSequence>
```

### 为什么“小陀螺区”不要写进 costmap

因为小陀螺是**行为**，不是导航代价：

- costmap 负责告诉机器人“是否可走 / 是否危险”
- BT 负责告诉机器人“到这里以后应该做什么”

因此：

- `keepout` → costmap
- `spin_zone` → behavior tree

---

## 和现有 RMUL_2026 行为树的结合方式

当前控制区逻辑已经存在，特别是高血量逻辑里的 B-1：

- 已到达控制区 → 站定 + 小陀螺 + KeepRunning

当前写法本质上还是“控制区中心点 + arrive_radius”的判定。这个逻辑是正确的，但表达方式偏几何目标点。

如果后续引入语义区，建议把“是否进入控制区”的判定逐步升级为：

```xml
<IsInSemanticZone zone_name="control_zone" .../>
```

而不是继续大量堆叠：

- `goal_x`
- `goal_y`
- `arrive_radius`

这样后续加多个不同区域时，行为树可维护性会更高。

---

## 动态语义区：第二阶段再做

当静态语义地图跑稳定后，再做动态语义更合理。

### 适合动态化的场景

1. 某条通道被临时判定不可通行
2. 敌方火力覆盖区临时升高代价
3. 队友已占位区域临时禁入
4. 某区域当前应切换为 spin / stop / search 模式

### 建议实现方式

新增一个状态话题，例如：

- `semantic_zone_state`

消息中描述各 zone 当前状态：

```yaml
zones:
  - name: control_zone
    active: true
    behavior: spin

  - name: lane_a
    active: true
    nav_mode: keepout

  - name: lane_b
    active: true
    nav_mode: high_cost
    cost: 220
```

然后：

1. `SemanticZoneLayer` 订阅它并动态改代价
2. BT 订阅它或由中间节点写黑板后再读取

这样“区域几何边界”与“区域当前状态”就能解耦。

---

## 更后面的第三阶段：感知驱动语义

这一步才开始接近真正的“在线语义导航”：

1. 视觉 / 雷达检测到火力扇区，临时生成危险区
2. 检测到队友占位，临时生成 keepout 区
3. 检测到拥堵区域，临时生成 high_cost 区

但这不建议作为第一步。

当前应先把：

1. 静态语义地图
2. 语义 costmap layer
3. BT 语义条件节点

这三件事做稳。

---

## 分阶段实施建议

### 第一阶段：最小可用版本

目标：让系统先支持“禁行区 + 小陀螺区”。

实施内容：

1. 新增 `semantic_zones.yaml`
2. 新增 `SemanticZoneLayer`
3. 新增 `IsInSemanticZone`
4. 先只标注以下几类区：
   - keepout
   - high_cost
   - control_zone / spin_zone

### 第二阶段：比赛可调版本

目标：支持比赛时动态调整区域规则。

实施内容：

1. 新增 `semantic_zone_state` topic
2. 支持 zone enable / disable
3. 支持 zone mode 动态切换
4. BT 与 costmap layer 同时消费状态

### 第三阶段：感知联动版本

目标：把视觉 / 雷达 / 规则系统的在线结果映射成语义区。

实施内容：

1. 在线危险区
2. 在线临时禁行区
3. 在线队友占位区
4. 在线拥堵高代价区

---

## 推荐的包与文件组织

### 导航侧

放在：

- `src/gxu2026_sentry_nav/pb_nav2_plugins/`

新增：

- `include/pb_nav2_plugins/layers/semantic_zone_layer.hpp`
- `src/layers/semantic_zone_layer.cpp`

### 行为树侧

放在：

- `src/RM_Behavior_Tree/rm_behavior_tree/plugins/rmul_2026/condition/`

新增：

- `is_in_semantic_zone.hpp`
- `is_in_semantic_zone.cpp`

### 配置侧

建议分仿真 / 实车：

- `src/gxu2026_sentry_nav/gxu2026_nav_bringup/config/simulation/semantic_zones.yaml`
- `src/gxu2026_sentry_nav/gxu2026_nav_bringup/config/reality/semantic_zones.yaml`

这样地图边界稍有差异时，配置层能独立维护。

---

## 对当前项目的最终建议

### 最优先做的事情

1. 先定义 `semantic_zones.yaml` 的格式
2. 先做 `SemanticZoneLayer`
3. 再做 `IsInSemanticZone`
4. 最后把控制区逻辑从“点 + 半径”逐步升级成“语义区”

### 不建议一开始就做的事情

1. 不要先上完整语义 SLAM 重构整套导航
2. 不要把“禁行”和“开小陀螺”混在一个节点里处理
3. 不要继续堆越来越多的 `goal_x + goal_y + arrive_radius` 来表达复杂区域

---

## 一句话总结

对当前 RMUL_2026 工程，最合适的路线不是“完整 semantic SLAM”，而是：

```text
已有地图 + 语义区域配置 + Nav2 语义代价层 + BT 语义条件节点
```

这样既能解决“哪里不能走”，也能解决“走到哪里要做什么”，并且能最大限度复用当前仓库已有的 Nav2 插件体系与 RMUL_2026 行为树结构。