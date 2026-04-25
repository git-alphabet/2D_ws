# fake_vel_transform

本功能包启动时，Fake Velocity Transform 会创建一个 `fake_robot_base_frame` 的坐标系，其 x, y, z 与 `robot_base_frame` 的坐标系一致，但 yaw 固定指向正前方，然后将生成的该坐标系发布到 tf2。同时，Fake Velocity Transform 会订阅 `input_cmd_vel_topic` 话题，将接收到的速度指令转换到 `robot_base_frame` 坐标系下，并发布到 `output_cmd_vel_topic` 话题。

主要目的是用于适配 NAV2 局部路径规划器，当速度参考坐标系 `robot_base_frame` 变化剧烈时，如云台处于自旋扫描时，NAV2 局部路径规划器会将机器人的方向视为与当前路径规划方向一致，导致机器人无法正常运动。而使用 `fake_robot_base_frame` 可以规避这个问题，实现较稳定的轨迹跟踪效果。

由于 NAV2 humble 发行版出于避免破坏原有接口的原因，依然使用 Twist 类型（不含时间戳），humble 往后的版本才使用 TwistStamped，导致无法直接实现 cmd_vel 与 odometry 的时间戳对齐。因此，本功能包暂时订阅 local_plan 话题（由局部路径规划器发布），以获取时间戳，将它的时间戳视为 cmd_vel 的时间戳，以间接实现时间戳对齐。
Related issue: [Switch from Twist to TwistStamped for cmd_vel #1594](https://github.com/ros-navigation/navigation2/issues/1594)

## Published Topics

* `tf` (`tf2_msgs/msg/TFMessage`) - 与机器人可移动关节相对应的变换
* `output_cmd_vel_topic` (`geometry_msgs/msg/Twist`) - 转换后的速度指令

## Subscribed Topics

* `input_cmd_vel_topic` (`geometry_msgs/msg/Twist`) - 机器人的速度指令
* `local_plan_topic` (`nav_msgs/msg/Path`) - 局部路径规划器的路径
* `odom_topic` (`nav_msgs/msg/Odometry`) - 里程计数据
* `robot_control_topic` (`rm_decision_interfaces/msg/RMUL`) - 小陀螺开关控制（读取 `chassis_spin`，true 时叠加匀速旋转）
* `manual_spin_override_topic` (`std_msgs/msg/Bool`) - 手动覆盖小陀螺开关（仅在 `use_manual_spin_override=true` 时生效，优先级高于 `robot_control_topic`）

## Parameters

* `odom_topic` (`string`, default: "odom") - 里程计话题。里程计的 frame_id 与 `robot_base_frame` 参数保持一致
* `robot_base_frame` (`string`, default: "gimbal_link") - 速度参考坐标系
* `fake_robot_base_frame` (`string`, default: "gimbal_link_fake") - 伪速度参考坐标系
* `local_plan_topic` (`string`, default: "local_plan") - 局部路径规划器的路径话题
* `robot_control_topic` (`string`, default: "robot_control") - 小陀螺开关控制话题（`rm_decision_interfaces/msg/RMUL`）
* `use_manual_spin_override` (`bool`, default: false) - 是否启用手动覆盖模式；启用后，忽略 `robot_control_topic` 中的 `chassis_spin`
* `manual_spin_override_topic` (`string`, default: "manual_chassis_spin") - 手动覆盖话题（`std_msgs/msg/Bool`）
* `input_cmd_vel_topic` (`string`, default: "") - 输入速度指令的话题
* `output_cmd_vel_topic` (`string`, default: "") - 输出速度指令的话题。将原本基于 `fake_robot_base_frame` 的速度变换到 `robot_base_frame` 后发布
* `init_spin_speed` (`double`, default: 0.0) - 匀速旋转角速度（`chassis_spin=true` 时叠加到 `output_cmd_vel_topic.angular.z`）
* `disable_spin_while_moving` (`bool`, default: true) - 移动过程中是否关闭小陀螺。`true`：线速度超过阈值时 `angular.z=0`；`false`：移动时仍叠加小陀螺，便于联调测试
* `enable_speed_bump_min_speed` (`bool`, default: true) - 是否启用减速带最小线速度限制
* `enable_speed_bump_zero_angular_z` (`bool`, default: true) - 是否在减速带区域强制 `angular.z=0`
* `speed_bump_min_linear_speed` (`double`, default: 1.5) - 减速带区域内的最小线速度（m/s）
* `speed_bump_map_frame` (`string`, default: "map") - 减速带多边形坐标系
* `speed_bump_zone_name` (`string`, default: "speed_bump") - 需要加载的减速带区域名称
* `speed_bump_zones_file` (`string`, default: "") - 减速带语义区域 YAML 路径

### 自旋来源优先级

`output_cmd_vel_topic.angular.z` 的小陀螺角速度来源优先级如下：

1. `cmd_spin_topic`（变速小陀螺实时值，收到后持续生效）
2. `init_spin_speed`（仅在尚未收到任何 `cmd_spin` 时作为回退值）
