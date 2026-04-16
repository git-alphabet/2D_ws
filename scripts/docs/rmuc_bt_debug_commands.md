# RMUC 2026 行为树 常用排查指令

> 仿真命名空间: `/red_standard_robot1`，以下命令均在 Docker 容器内执行。

## 1. BT 进程状态

```bash
# BT进程是否存在
ps aux | grep rm_behavior_tree | grep -v grep

# BT ROS节点是否注册
ros2 node list | grep -i rmuc
```

## 2. 话题数据验证

```bash
# 机器人位置（pose.x/y 是否正常）
ros2 topic echo /red_standard_robot1/robot_position --once

# 裁判系统状态（game_progress / hp）
ros2 topic echo /red_standard_robot1/game_status --once
ros2 topic echo /red_standard_robot1/robot_status --once

# 底盘控制（chassis_spin 是否为 True/False）
ros2 topic echo /red_standard_robot1/robot_control --once

# 所有话题列表
ros2 topic list | grep red_standard_robot1 | sort
```

## 3. 导航状态

```bash
# 当前导航目标反馈
ros2 topic echo /red_standard_robot1/navigate_to_pose/_action/feedback --once 2>/dev/null || echo "无活跃导航"

# goal_pose 话题
ros2 topic echo /red_standard_robot1/goal_pose --once --timeout 3
```

## 4. BT 日志

```bash
# 搜索日志文件中的 BT 关键信息
grep -i "behavior_tree\|rmuc\|IsAtGoal\|game_progress\|hp_below\|LowHP" /ws/launch_logs/*.log 2>/dev/null | tail -30

# 最新日志中的错误
ls -lt /ws/launch_logs/ | head -5
cat /ws/launch_logs/$(ls -t /ws/launch_logs/ | head -1) | grep -i "error\|crash\|behavior\|abort" | tail -20
```

## 5. 手动启动 BT（调试时）

```bash
source /ws/.buildcache/Alphabet/install/setup.bash && \
ros2 run rm_behavior_tree rm_behavior_tree_rmuc --ros-args \
  -p style:=/ws/.buildcache/Alphabet/install/rm_behavior_tree/share/rm_behavior_tree/config/RMUC_2026/rmuc_2026.xml \
  --params-file /ws/src/gxu2026_sentry_nav/gxu2026_nav_bringup/config/simulation/nav2_params.yaml \
  -r __ns:=/red_standard_robot1
```

## 6. 重启 BT 进程

```bash
# 杀掉旧 BT 和 bridge
kill $(pgrep -f rm_behavior_tree_rmuc) $(pgrep -f robot_position_bridge) 2>/dev/null
# launch 的 respawn 机制会自动重启
```

## 7. TF 树（仿真带命名空间）

```bash
ros2 run rqt_tf_tree rqt_tf_tree --ros-args -r /tf:=tf -r /tf_static:=tf_static -r __ns:=/red_standard_robot1
```

## 8. 编译 BT（使用 buildcache 路径）

```bash
cd /ws && source /ws/.buildcache/Alphabet/install/setup.bash && \
colcon build --build-base /ws/.buildcache/Alphabet/build \
             --install-base /ws/.buildcache/Alphabet/install \
             --base-paths /ws/src/RM_Behavior_Tree/rm_behavior_tree \
             --packages-select rm_behavior_tree
```

## 9. 姿态转换相关调试

> 姿态值: 1=进攻  2=防御  3=移动
> 决策链: 裁判系统 → `sentry_decision_status` → `ParseSentryBlackboard` → `DecidePosture` → `SentryCmdMux` → `/sentry_cmd`

### 9.1 查看当前姿态反馈（裁判系统 → BT 输入）

```bash
# 裁判系统反馈的当前姿态 (current_posture 字段)
ros2 topic echo /red_standard_robot1/sentry_decision_status --field current_posture

# 完整 sentry_decision_status（含复活/兑换计数等）
ros2 topic echo /red_standard_robot1/sentry_decision_status --once
```

### 9.2 查看 BT 输出的姿态指令（BT → 电控）

```bash
# 仅看姿态指令字段
ros2 topic echo /red_standard_robot1/sentry_cmd --field cmd_posture

# 完整 sentry_cmd（含复活/远程兑换/允许发弹量等）
ros2 topic echo /red_standard_robot1/sentry_cmd --once
```

### 9.3 姿态决策影响因子（DecidePosture 的输入）

```bash
# 血量（hp_cur / hp_max 影响攻防评分）
ros2 topic echo /red_standard_robot1/robot_status --once --field current_hp
ros2 topic echo /red_standard_robot1/robot_status --once --field hp_max

# 枪管热量（heat_cur > heat_high 时偏向防御）
ros2 topic echo /red_standard_robot1/robot_status --once --field shooter_heat

# 是否检测到目标（has_target → 攻击评分 +35）
ros2 topic echo /red_standard_robot1/robot_status --once --field detect_enemy

# buff 状态（防御增益/易伤标记/冷却增益影响评分）
ros2 topic echo /red_standard_robot1/robot_buff --once

# 允许发弹量（ammo<=0 时攻击评分归零，移动评分 +50）
ros2 topic echo /red_standard_robot1/projectile_allowance --once
```

### 9.4 姿态转换对比监控（同时观察输入姿态 vs 输出姿态）

```bash
# 终端1: 裁判系统当前姿态（输入）
ros2 topic echo /red_standard_robot1/sentry_decision_status --field current_posture

# 终端2: BT 决策后的姿态指令（输出）
ros2 topic echo /red_standard_robot1/sentry_cmd --field cmd_posture
```

### 9.5 姿态转换日志

```bash
# 搜索 BT 日志中的姿态相关输出
grep -i "posture\|姿态\|DecidePosture" /ws/launch_logs/*.log 2>/dev/null | tail -30

# 搜索 sentry_cmd 发布相关
grep -i "SentryCmdMux\|sentry_cmd" /ws/launch_logs/*.log 2>/dev/null | tail -20
```

### 9.6 姿态决策评分逻辑速查

| 条件 | 攻击(1) | 防御(2) | 移动(3) |
|------|---------|---------|---------|
| 基础分 | 5 | 8 | 10 |
| has_target | +35 | — | — |
| hp > 50% | +20 | — | — |
| 30% < hp < 50% | +8 | +15 | +5 |
| hp < 30% | — | +35 | — |
| buff_defense > 0 | +15 | — | — |
| buff_cool > 0 | +12 | — | — |
| buff_vuln > 0 | — | +50 | — |
| base_threat | — | +30 | — |
| heat > heat_high | — | +25 | — |
| !has_target | — | +8 | +10 |
| elapsed > 180s | +18 | — | — |
| ammo <= 0 | →0 | — | +50 |

> **切换规则**: 5秒冷却 + 新姿态评分需超出当前评分 ≥12 分才切换。`base_threat` 强制切换为攻击(1)且无视冷却。
