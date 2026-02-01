#!/usr/bin/env bash
set -euo pipefail

# 诊断目标：
# 1) /auto_aim_yaw -> auto_aim_yaw_joint_state_bridge -> /serial/gimbal_joint_state
# 2) joint_state_publisher 是否订阅 /serial/gimbal_joint_state 并输出 /joint_states
# 3) robot_state_publisher 是否基于 /joint_states 生成 base_frame->gimbal_yaw 的 TF（yaw 是否随输入变化）
# 4) 是否存在“namespace 不一致”导致发布/订阅对不上

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Source ROS + overlay (nounset-friendly)
set +u
if [ -f /opt/ros/humble/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi
if [ -f "${WS_ROOT}/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${WS_ROOT}/install/setup.bash"
fi
set -u

# Let ros2 CLI resolve sp_msgs/msg/Float32Stamped
export AMENT_PREFIX_PATH="${WS_ROOT}/install/sp_msgs:${AMENT_PREFIX_PATH:-}"

say() { echo -e "\n==== $* ===="; }

say "Env quickcheck"
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<unset>}"
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-<unset>}"

say "Package prefixes (what will be used if launched now)"
ros2 pkg prefix joint_state_publisher 2>/dev/null || true
ros2 pkg prefix pb2025_robot_description 2>/dev/null || true
ros2 pkg prefix gimbal_yaw_bridge 2>/dev/null || true

say "Processes (who actually started jsp/rsp)"
ps -ef | grep -E "joint_state_publisher(/joint_state_publisher)?( |$)|robot_state_publisher( |$)" | grep -v grep || true
ps -ef | grep -E "ros2 launch|launch.py" | grep -v grep | head -n 30 || true

say "Nodes (filter)"
ros2 node list | grep -E "auto_aim_yaw_joint_state_bridge|joint_state_publisher|robot_state_publisher" || true

say "Topics (filter)"
ros2 topic list | grep -E "/auto_aim_yaw|/serial/gimbal_joint_state|/joint_states|tf_static|tf$" || true

say "Topic info: /auto_aim_yaw"
ros2 topic info /auto_aim_yaw -v || true

say "Topic hz (2s): /auto_aim_yaw"
(timeout -s INT 2 ros2 topic hz /auto_aim_yaw 2>/dev/null) || true

say "Topic info: /serial/gimbal_joint_state (expect: sub>=1 if joint_state_publisher is consuming)"
ros2 topic info /serial/gimbal_joint_state -v || true

say "Topic hz (2s): /serial/gimbal_joint_state"
(timeout -s INT 2 ros2 topic hz /serial/gimbal_joint_state 2>/dev/null) || true

say "Topic info: /red_standard_robot1/serial/gimbal_joint_state (namespace variant, if any)"
ros2 topic info /red_standard_robot1/serial/gimbal_joint_state -v || true

say "Topic info: /joint_states (expect: pub>=1 if joint_state_publisher is running)"
ros2 topic info /joint_states -v || true

say "Topic hz (2s): /joint_states"
(timeout -s INT 2 ros2 topic hz /joint_states 2>/dev/null) || true

say "Node info: /joint_state_publisher (if exists)"
ros2 node info /joint_state_publisher || true

say "Param: /joint_state_publisher rate"
ros2 param get /joint_state_publisher rate || true

say "Param: /joint_state_publisher source_list (expect includes serial/gimbal_joint_state)"
ros2 param get /joint_state_publisher source_list || true

say "Param dump: /joint_state_publisher (head)"
ros2 param dump /joint_state_publisher 2>/dev/null | head -n 80 || true

say "Node info: /robot_state_publisher (if exists)"
ros2 node info /robot_state_publisher || true

say "TF echo: base_footprint -> gimbal_yaw (2s)"
# tf2_echo 会持续输出，这里用 timeout 截取一小段
(timeout 2 ros2 run tf2_ros tf2_echo base_footprint gimbal_yaw) || true

say "TF echo: chassis -> gimbal_yaw (2s)"
(timeout 2 ros2 run tf2_ros tf2_echo chassis gimbal_yaw) || true

say "Done"
