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
