#!/usr/bin/env bash
set -euo pipefail

# Diagnose the chain:
# /auto_aim_yaw -> auto_aim_yaw_joint_state_bridge -> /serial/gimbal_joint_state -> joint_state_publisher -> /joint_states -> robot_state_publisher -> TF

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

echo "[diag] workspace: ${WS_ROOT}"

# Source ROS + overlay. Keep nounset-friendly.
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

echo ""
echo "=== Nodes (state publishers) ==="
ros2 node list | grep -E "joint_state_publisher|robot_state_publisher" || true

echo ""
echo "=== Node details: /joint_state_publisher ==="
if ros2 node list | grep -qx "/joint_state_publisher"; then
  ros2 node info /joint_state_publisher || true
  echo "[diag] /joint_state_publisher params (key ones):"
  ros2 param get /joint_state_publisher source_list || true
  ros2 param get /joint_state_publisher rate || true
else
  echo "[diag] /joint_state_publisher not found"
fi

echo ""
echo "=== Node details: robot_state_publisher (if any) ==="
ros2 node list | grep -E "/robot_state_publisher$" || true

echo ""
echo "=== Nodes (auto_aim_yaw bridge instances) ==="
ros2 node list | grep -E "auto_aim_yaw_joint_state_bridge" || true

echo ""
echo "[diag] If you see multiple '/auto_aim_yaw_joint_state_bridge' lines above, it means you have multiple instances running (not recommended)."
echo "[diag] You can force it down to ONE instance via: ./scripts/run_single_auto_aim_yaw_bridge.sh"

echo ""
echo "=== Topics (gimbal/joint_state related) ==="
ros2 topic list | grep -E "gimbal_joint_state|joint_states|auto_aim_yaw" || true

echo ""
echo "=== Topic info: /auto_aim_yaw ==="
ros2 topic info -v /auto_aim_yaw || true

echo ""
echo "=== Topic info: /serial/gimbal_joint_state ==="
ros2 topic info -v /serial/gimbal_joint_state || true

echo ""
echo "=== Topic info: /joint_states ==="
ros2 topic info -v /joint_states || true

echo ""
echo "=== TF publishers (topic: /tf) ==="
ros2 topic info -v /tf || true

echo ""
echo "=== TF_STATIC publishers (topic: /tf_static) ==="
ros2 topic info -v /tf_static || true

echo ""
echo "=== Try echo TF (3s timeout): base_footprint -> gimbal_yaw ==="
if command -v timeout >/dev/null 2>&1; then
  timeout 3 ros2 run tf2_ros tf2_echo base_footprint gimbal_yaw || true
else
  echo "[diag] 'timeout' not found; running tf2_echo without timeout (Ctrl+C to stop)"
  ros2 run tf2_ros tf2_echo base_footprint gimbal_yaw || true
fi

echo ""
echo "[diag] Next checks (manual):"
echo "  1) If /serial/gimbal_joint_state has 0 subscribers, locate joint_state_publisher subscriptions via:"
echo "     ros2 node info /joint_state_publisher"
echo "  2) If robot_state_publisher subscribes /joint_states, but yaw still does not change:"
echo "     publish a test /auto_aim_yaw and observe tf2_echo chassis gimbal_yaw"
