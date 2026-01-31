#!/usr/bin/env bash
set -euo pipefail

# Ensure there is ONLY ONE auto_aim_yaw_joint_state_bridge instance.
# It will:
# 1) source ROS + overlay
# 2) kill all existing auto_aim_yaw_joint_state_bridge processes
# 3) start exactly one instance with the canonical topics

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

usage() {
  cat <<'EOF'
Usage:
  run_single_auto_aim_yaw_bridge.sh [--kill-only]

Options:
  --kill-only   Only kill existing bridge processes, do not restart.

Notes:
  - input_topic is absolute: /auto_aim_yaw
  - output_topic is relative: serial/gimbal_joint_state
EOF
}

KILL_ONLY=0
if [ "${1:-}" = "-h" ] || [ "${1:-}" = "--help" ]; then
  usage
  exit 0
fi
if [ "${1:-}" = "--kill-only" ]; then
  KILL_ONLY=1
fi

echo "[bridge-one] workspace: ${WS_ROOT}"

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

# Kill all existing bridge processes.
PIDS="$(pgrep -f "(^|/)auto_aim_yaw_joint_state_bridge(\\s|$)" || true)"
if [ -n "${PIDS}" ]; then
  echo "[bridge-one] killing existing auto_aim_yaw_joint_state_bridge pids: ${PIDS}"
  # shellcheck disable=SC2086
  kill ${PIDS} || true
  sleep 0.3
  PIDS2="$(pgrep -f "(^|/)auto_aim_yaw_joint_state_bridge(\\s|$)" || true)"
  if [ -n "${PIDS2}" ]; then
    echo "[bridge-one] force-killing remaining pids: ${PIDS2}"
    # shellcheck disable=SC2086
    kill -9 ${PIDS2} || true
  fi
else
  echo "[bridge-one] no existing bridge process found"
fi

if [ "${KILL_ONLY}" -eq 1 ]; then
  echo "[bridge-one] --kill-only done"
  exit 0
fi

echo "[bridge-one] starting ONE bridge instance..."
exec ros2 run gimbal_yaw_bridge auto_aim_yaw_joint_state_bridge --ros-args \
  -p input_topic:=/auto_aim_yaw \
  -p output_topic:=serial/gimbal_joint_state
