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

# 注意：/red_standard_robot1/robot_status 是裁判系统/测试脚本发布的原始输入。
# 语义区“忽略敌人”只覆盖 BT 黑板里的有效 is_detect_enemy，不会反向改写这个原始话题。

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
> 当前 RMUC 2026 主链: 各战术/全局控制子树写入 `cmd.posture` → `PostureDegradationGuard` 产出 `cmd.final_posture` → `SentryCmdMux` 发布 `/sentry_cmd`
> `sentry_decision_status` 已归档，当前 BT 不订阅、不发布；姿态查询以 `/sentry_cmd.cmd_posture` 为准。

### 9.1 查看 BT 输出的姿态指令（BT → 电控/裁判系统）

```bash
# 仅看姿态指令字段
ros2 topic echo /red_standard_robot1/sentry_cmd --field cmd_posture

# 完整 sentry_cmd（当前包含姿态与确认复活）
ros2 topic echo /red_standard_robot1/sentry_cmd --once
```

### 9.2 手动只发布 sentry_cmd（不发布其他裁判输入）

```bash
source /opt/ros/humble/setup.bash && \
source /ws/.buildcache/Alphabet/install/setup.bash

# 姿态 1=进攻
python3 /ws/scripts/rmuc_test_publisher.py --ns=/red_standard_robot1 --only=sentry_cmd --posture=1

# 姿态 2=防御
python3 /ws/scripts/rmuc_test_publisher.py --ns=/red_standard_robot1 --only=sentry_cmd --posture=2

# 姿态 3=移动
python3 /ws/scripts/rmuc_test_publisher.py --ns=/red_standard_robot1 --only=sentry_cmd --posture=3

# 确认复活
python3 /ws/scripts/rmuc_test_publisher.py --ns=/red_standard_robot1 --only=sentry_cmd --confirm-respawn

# 指定姿态并同时确认复活
python3 /ws/scripts/rmuc_test_publisher.py --ns=/red_standard_robot1 --only=sentry_cmd --posture=3 --confirm-respawn
```

> 注意：这会额外发布 `/red_standard_robot1/sentry_cmd`。如果 BT 同时运行，它也会发布同一个话题，实际接收端会看到两路发布者混在一起。需要单独测电控/裁判接收姿态时，建议暂停 BT 或用 `ros2 topic info /red_standard_robot1/sentry_cmd -v` 确认发布者数量。

### 9.3 姿态决策影响因子（当前 BT 原始输入）

```bash
# 血量：低血触发 LowHPRetreat，输出移动(3)并回补给区
ros2 topic echo /red_standard_robot1/robot_status --once --field current_hp

# 弹药：低弹触发 SustainAndEconomy，输出防御(2)并走补给链路
ros2 topic echo /red_standard_robot1/robot_status --once --field ammo_allow

# 是否检测到敌人：普通敌情分支按 enemy_hold_attack_enable 切姿态
ros2 topic echo /red_standard_robot1/robot_status --once --field is_detect_enemy

# 基地血量与雷达敌情：共同影响基地威胁锁存
ros2 topic echo /red_standard_robot1/robot_status --once --field base_hp
ros2 topic echo /red_standard_robot1/radar/enemy_tracks --once

# 己方前哨站存活状态：影响目标点选择，进而影响到达目标后的防御/移动姿态
ros2 topic echo /red_standard_robot1/robot_status --once --field outpost_hp
```

### 9.4 姿态转换对比监控

```bash
# 终端1: BT 最终姿态输出
ros2 topic echo /red_standard_robot1/sentry_cmd --field cmd_posture

# 终端2: 当前 active_subtree 只能通过日志/Groot 侧看；
# CLI 侧通常结合以下输入与 sentry_cmd 输出判断是哪条分支接管。
ros2 topic echo /red_standard_robot1/robot_status --once
ros2 topic echo /red_standard_robot1/robot_position --once
```

### 9.5 姿态转换日志

```bash
# 搜索 BT 日志中的姿态相关输出
grep -i "posture\|姿态\|DecidePosture" /ws/launch_logs/*.log 2>/dev/null | tail -30

# 搜索 sentry_cmd 发布相关
grep -i "SentryCmdMux\|sentry_cmd" /ws/launch_logs/*.log 2>/dev/null | tail -20
```
> **切换规则**: 普通姿态切换仍受 5 秒冷却和姿态降级守卫约束。
> **普通见敌模式**: `enemy_hold_attack_enable=true` 时普通见敌切攻击(1)，`false` 时切防御(2)，同时取消导航并原地自转。

## 10. 基地威胁模式调试

```bash
# 1) 单独开一个终端，发送基地威胁输入：敌人在基地附近 + 基地持续掉血
source /opt/ros/humble/setup.bash && \
source /ws/.buildcache/Alphabet/install/setup.bash && \
python3 /ws/scripts/used/rmuc_test_publisher.py \
  --phase=4 \
  --enemy-near-base \
  --base-hp-drain=5 \
  --duration=25

# 3) 观察姿态：早期应为移动(3)，到达防御锚点并检测到敌人后应切到防御(2)
ros2 topic echo /red_standard_robot1/sentry_cmd --field cmd_posture

# 3.1) 查询当前基地威胁锁存状态（最关键）
# 当前没有单独 ROS 话题直接发布 threat.base；
# 看最新一条基地威胁事件日志即可判断当前状态：
# - 最新是“基地威胁触发” → 当前 threat.base=true
# - 最新是“基地威胁解除” → 当前 threat.base=false
grep -hE '基地威胁触发|基地威胁解除' /ws/launch_logs/*.log 2>/dev/null | tail -1

# 4) 观察 BT 是否发出回防锚点目标
grep -i 'DefendAnchor\|New goal\|Reached the goal' /ws/launch_logs/*.log 2>/dev/null | tail -20

# 5) 观察当前位置是否接近 defend_anchor
ros2 topic echo /red_standard_robot1/robot_position --once
```

> 预期现象:
> 0. `grep ... | tail -1` 若输出“基地威胁触发”，说明当前 `threat.base=true`；若最后一条是“基地威胁解除”，说明当前 `threat.base=false`。
> 1. `/red_standard_robot1/sentry_cmd.cmd_posture` 先为 3，随后变为 2。
> 2. 日志出现 `DefendAnchor New goal: [ 0.097, -0.401 ]`。
> 3. 控制器日志出现 `Reached the goal!`。

## 11. 语义区忽略敌人调试

```bash
# 1) 确认原始裁判输入：这里为 true 是正常的，说明测试脚本确实模拟“扫到敌人”
ros2 topic echo /red_standard_robot1/robot_status --once --field is_detect_enemy

# 2) 确认当前位置是否落在 semantic_zones.yaml 的 speed_bump 多边形内
ros2 topic echo /red_standard_robot1/robot_position --once
cat /ws/src/gxu2026_sentry_nav/gxu2026_nav_bringup/config/simulation/semantic_zones.yaml

# 3) 看 BT 是否启用语义敌情覆盖
grep -h "semantic enemy override" /ws/launch_logs/*.log 2>/dev/null | tail -5

# 4) 观察行为效果：语义区内即使 robot_status.is_detect_enemy=true，
# EnemyHold 分支也不应因为见敌接管；重点看是否仍持续导航、robot_control 是否被敌情分支强制 spin。
ros2 topic echo /red_standard_robot1/robot_control --once
ros2 topic echo /red_standard_robot1/nav_control_cmd --once --timeout 2
```

> 预期现象:
> 1. `/red_standard_robot1/robot_status.is_detect_enemy` 仍可为 `true`，这是原始输入。
> 2. 进入 speed_bump 后日志出现 `semantic enemy override enabled`。
> 3. 离开 speed_bump 后日志出现 `semantic enemy override disabled`，BT 恢复使用原始 `is_detect_enemy`。
