#!/usr/bin/env bash
set -euo pipefail

# 诊断目标：
# 1) /auto_aim_yaw -> auto_aim_yaw_joint_state_bridge -> /serial/gimbal_joint_state
# 2) joint_state_publisher 是否订阅 /serial/gimbal_joint_state 并输出 /joint_states
# 3) robot_state_publisher 是否基于 /joint_states 生成 base_frame->gimbal_yaw 的 TF（yaw 是否随输入变化）
# 4) 是否存在“namespace 不一致”导致发布/订阅对不上

_d="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
while [[ "$_d" != "/" && ! -f "$_d/install/setup.bash" ]]; do _d="$(dirname "$_d")"; done
WS_ROOT="$_d"; unset _d

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

# 当 /auto_aim_yaw 没有任何 publisher 时，自动做一次短时自测发布。
# 关闭方法：AUTO_TEST_AUTO_AIM_YAW=0 ./scripts/diagnose_gimbal_yaw_chain.sh
AUTO_TEST_AUTO_AIM_YAW="${AUTO_TEST_AUTO_AIM_YAW:-1}"
AUTO_TEST_PID=""

cleanup() {
  if [[ -n "${AUTO_TEST_PID:-}" ]]; then
    kill "${AUTO_TEST_PID}" 2>/dev/null || true
    wait "${AUTO_TEST_PID}" 2>/dev/null || true
  fi
}

trap cleanup EXIT INT TERM

auto_test_auto_aim_yaw_if_needed() {
  local info pub_count
  info="$(ros2 topic info /auto_aim_yaw -v 2>/dev/null || true)"
  echo "$info"
  pub_count="$(echo "$info" | awk -F': ' '/Publisher count/ {print $2; exit}' | tr -d '\r')"

  if [[ "$AUTO_TEST_AUTO_AIM_YAW" == "0" ]]; then
    return 0
  fi

  if [[ -z "$pub_count" || "$pub_count" == "0" ]]; then
    say "Auto self-test: publish /auto_aim_yaw (Publisher count is 0)"
    echo "AUTO_TEST: Publishing sp_msgs/msg/Float32Stamped -> /auto_aim_yaw for a few seconds"

    # 优先使用单个 rclpy publisher 发布一段时间（避免 ros2cli 频繁起进程导致 node 重名警告）
    (
      set +e
      python3 - <<'PY'
import os
import time

import rclpy
from rclpy.node import Node

try:
    from sp_msgs.msg import Float32Stamped
except Exception as e:
    raise SystemExit(f"Failed to import sp_msgs.msg.Float32Stamped: {e}")


class Pub(Node):
    def __init__(self):
        super().__init__(f"auto_aim_yaw_selftest_{os.getpid()}")
        self.pub = self.create_publisher(Float32Stamped, "/auto_aim_yaw", 10)


def main():
    rclpy.init()
    node = Pub()
    values = [0.0, 0.6, -0.6, 1.2, -1.2]
    start = time.time()
    i = 0
    while time.time() - start < 4.0:
        msg = Float32Stamped()
        msg.data = float(values[i % len(values)])
        # 给一个有效时间戳（bridge 也能处理 0 时间戳，但这里更直观）
        msg.header.stamp = node.get_clock().now().to_msg()
        node.pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(0.02)  # 50Hz
        i += 1
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
PY
      rc=$?
      if [[ $rc -ne 0 ]]; then
        echo "AUTO_TEST: rclpy publisher failed (rc=$rc), falling back to ros2 topic pub --once loop" >&2
        local values=(0.0 0.6 -0.6 1.2 -1.2)
        local i v
        for i in $(seq 1 40); do
          v="${values[$((i % ${#values[@]}))]}"
          ros2 topic pub --once /auto_aim_yaw sp_msgs/msg/Float32Stamped "{data: ${v}}" \
            >/dev/null 2>&1 || true
          sleep 0.1
        done
      fi
      set -e
    ) &
    AUTO_TEST_PID="$!"
    # 给 ROS graph 一点时间更新端点统计
    sleep 0.5
  fi
}

say "Env quickcheck"
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<unset>}"
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-<unset>}"

say "Package prefixes (what will be used if launched now)"
ros2 pkg prefix joint_state_publisher 2>/dev/null || true
ros2 pkg prefix gxu2026_robot_description 2>/dev/null || true
ros2 pkg prefix gimbal_yaw_bridge 2>/dev/null || true

say "Processes (who actually started jsp/rsp)"
ps -ef | grep -E "joint_state_publisher(/joint_state_publisher)?( |$)|robot_state_publisher( |$)" | grep -v grep || true
ps -ef | grep -E "ros2 launch|launch.py" | grep -v grep | head -n 30 || true

say "Nodes (filter)"
ros2 node list | grep -E "auto_aim_yaw_joint_state_bridge|joint_state_publisher|robot_state_publisher" || true

say "Topics (filter)"
ros2 topic list | grep -E "/auto_aim_yaw|/serial/gimbal_joint_state|/joint_states|tf_static|tf$" || true

say "Topic info: /auto_aim_yaw"
auto_test_auto_aim_yaw_if_needed

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

cleanup
