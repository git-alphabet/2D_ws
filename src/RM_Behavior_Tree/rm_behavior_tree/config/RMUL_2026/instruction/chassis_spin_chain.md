## 完整链路：底盘小陀螺角速度是怎么发出去的

### 总览（5层）

```
BT XML
  └─ RobotControl 节点
       └─ /robot_control (sp_msgs/RMUL, chassis_spin=true)
            ├─ nonlinear_spin_publisher (订阅)
            │    └─ /cmd_spin (Float32, rad/s) → 100Hz
            │         └─ fake_vel_transform (订阅)
            │              └─ /cmd_vel (Twist) → angular.z 加上 spin_speed_
            └─ fake_vel_transform (订阅, spin_enabled_ 开关)
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

### 第三层：nonlinear_spin_publisher 生成角速度

订阅 `robot_control`，通过 `onRobotControl` 回调设置内部标志：

```cpp
void onRobotControl(const sp_msgs::msg::RMUL::ConstSharedPtr msg) {
    chassis_spin_enabled_ = msg->chassis_spin;
}
```

100Hz 定时器 `onTimer()` 运行：
- `chassis_spin_enabled_ = false` → 发布 `0.0` rad/s，停转
- `chassis_spin_enabled_ = true` → 执行非线性随机角速度生成，发布到 `/cmd_spin`（`Float32`）

---

### 第四层：角速度数值是怎么算出来的

非线性目标生成（`retarget()` 函数）：

**公式**：
$$w_\text{target} = \text{center\_speed} + \text{range\_speed} \times U(-1, 1)$$

**nav2_params.yaml 中的实际参数**：
| 参数 | 值 | 含义 |
|---|---|---|
| `center_speed` | **3.14 rad/s** | 角速度中心值（≈π rad/s ≈ 0.5圈/s） |
| `range_speed` | **2.0 rad/s** | 随机扰动范围 |
| `max_abs_speed` | **6.28 rad/s** | 最高限幅（≈2π rad/s ≈ 1圈/s） |
| `min_abs_speed` | **2.0 rad/s** | 最低限幅（避免转速接近0） |
| `target_update_period` | **0.15s** | 每 150ms 重新抽一个目标速度 |
| `accel_limit` | **15.0 rad/s²** | 加速度限幅，控制变化平滑度 |

所以 `w_target` 范围是 **1.14 ~ 5.14 rad/s**（3.14 ± 2.0）。每个 timer 步长按加速度限幅渐变到目标值：

$$w_\text{current} += \text{clamp}(w_\text{target} - w_\text{current}, -\text{accel\_limit} \times dt, +\text{accel\_limit} \times dt)$$

---

### 第五层：fake_vel_transform 叠加到 cmd_vel

`transformVelocity()`:

```cpp
if (has_received_cmd_spin_) {
    current_spin = spin_speed_;   // 来自 /cmd_spin 订阅回调
} else {
    current_spin = init_spin_speed_;  // 默认 0.0
}
aft_tf_vel.angular.z = twist->angular.z + current_spin;  // 叠加到原始 cmd_vel
```

`spin_enabled_`（来自 `robot_control`）在 `fake_vel_transform` 里**不再作为开关**——开关逻辑交给 `nonlinear_spin_publisher`，它在 `chassis_spin=false` 时主动发布 `0.0`，`fake_vel_transform` 直接把这个 `0.0` 叠加即可。

---

### 停下来转的完整时序

```
BT: chassis_spin=True 写入 XML
  → RobotControl.setMessage() 发出 robot_control{chassis_spin=true}
  → nonlinear_spin_publisher.onRobotControl(): chassis_spin_enabled_=true
  → onTimer() 每 10ms: 生成 w_current ∈ [2.0, 6.28] rad/s, 发布 /cmd_spin
  → fake_vel_transform.cmdSpinCallback(): spin_speed_=w_current
  → 同时 NavControlCmd cmd_type=3 → Nav2 发出 cmd_vel.linear=0（刹车）
  → transformVelocity(): angular.z = 0 + spin_speed_ = w_current
  → /cmd_vel 发布：linear全0 + angular.z ≈ 3.14 rad/s（随机抖动）
```

机器人原地不动（线速度为0）但底盘旋转（角速度 ≈ π~2π rad/s），这就是小陀螺。
