## 完整链路：底盘小陀螺角速度是怎么发出去的

> **当前方案**：方案1 — 停车时由 `fake_vel_transform` 叠加固定角速度，移动时不叠加（Nav2 控制器全权负责）。
> `nonlinear_spin_publisher` 节点已**禁用**，不再参与调用链。

### 总览（3层）

```
BT XML
  └─ RobotControl 节点
       └─ /robot_control (sp_msgs/RMUL, chassis_spin=true/false)
            └─ fake_vel_transform（订阅，spin_enabled_ 开关）
                 ├─ 停车时（linear < 0.05 m/s）：angular.z += init_spin_speed
                 └─ 移动时（linear ≥ 0.05 m/s）：angular.z 不叠加，由 Nav2 控制器决定
```

---

### 第一层：BT XML 触发决策

[HighHP_combat_logic.xml](../HighHP_combat_logic.xml) 中，三个场景都会写 `chassis_spin="True"`：
- **检测到敌人（A-1/A-2）**：开小陀螺
- **到达控制区（B-1）**：开小陀螺
- **到达脱困点（B-2）**：开小陀螺

导航移动（B-3）时写 `chassis_spin="False"`。

---

### 第二层：RobotControl BT节点发话题

[robot_control.cpp](../../../../plugins/rmul_2026/action/robot_control.cpp) 实现非常简单：

```cpp
bool RobotControlAction::setMessage(sp_msgs::msg::RMUL & msg)
{
    msg.chassis_spin = false;
    getInput("chassis_spin", msg.chassis_spin);  // 从 XML 属性读值
    return true;
}
```

发布到 `robot_control` 话题（`sp_msgs/msg/RMUL`，字段 `chassis_spin = true/false`）。这个话题是**开关信号**，不含角速度值。

---

### 第三层：fake_vel_transform 判断并叠加角速度

`robotControlCallback` 收到消息后把 `spin_enabled_` 置为对应值。

`transformVelocity()` 每次处理 Nav2 速度指令时执行双重判断：

```cpp
constexpr double SPIN_LINEAR_STOP_THRESHOLD = 0.05;  // m/s

const bool effective_spin_enabled =
    use_manual_spin_override_ ? manual_spin_override_enabled_ : spin_enabled_;
const bool is_chassis_stationary =
    std::hypot(twist->linear.x, twist->linear.y) < SPIN_LINEAR_STOP_THRESHOLD;
const float current_spin =
    (effective_spin_enabled && is_chassis_stationary) ? init_spin_speed_ : 0.0f;
aft_tf_vel.angular.z = twist->angular.z + current_spin;
```

**两个条件必须同时满足才叠加**：
| 条件 | 含义 |
|---|---|
| `effective_spin_enabled == true` | 行为树开启了小陀螺（`chassis_spin=True`） |
| `is_chassis_stationary == true` | 底盘线速度 < 0.05 m/s，视为静止 |

**`init_spin_speed` 参数值**：
| 环境 | 值 | 配置文件 |
|---|---|---|
| 仿真 | **3.14 rad/s**（≈ π rad/s） | `simulation/nav2_params.yaml` |
| 实车 | **2.0 rad/s** | `reality/nav2_params.yaml` |

---

### 停下来转的完整时序

```
BT: chassis_spin=True 写入 XML
  → RobotControl.setMessage() 发出 robot_control{chassis_spin=true}
  → fake_vel_transform.robotControlCallback(): spin_enabled_=true
  → 同时 NavControlCmd cmd_type=3 → Nav2 控制器刹车
       → cmd_vel_nav2_result.linear.x ≈ 0, linear.y ≈ 0
  → transformVelocity():
       effective_spin_enabled=true
       is_chassis_stationary=true（线速度 < 0.05 m/s）
       current_spin = init_spin_speed_（仿真3.14 / 实车2.0 rad/s）
       angular.z = 0 + current_spin
  → /cmd_vel 发布：linear全0 + angular.z = init_spin_speed → 原地小陀螺
```

### 移动时不干扰导航的时序

```
BT: chassis_spin=True（但正在导航移动）
  → spin_enabled_=true
  → Nav2 控制器正常输出线速度，linear ≥ 0.05 m/s
  → transformVelocity():
       effective_spin_enabled=true
       is_chassis_stationary=false（线速度 ≥ 0.05 m/s）
       current_spin = 0.0f
       angular.z = twist->angular.z + 0（Nav2 控制器全权决定）
  → /cmd_vel 发布：Nav2 正常导航速度，不叠加自旋
```

机器人停下时原地旋转（小陀螺防御），移动时角速度由 Nav2 路径跟踪控制器完全接管，两者互不干扰。
