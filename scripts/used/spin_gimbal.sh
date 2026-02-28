#!/usr/bin/env bash
set -euo pipefail

# 用法：
#   ./scripts/spin_gimbal.sh [/red_standard_robot1] [yaw_step_rad] [rate_hz]
# 示例：
#   ./scripts/spin_gimbal.sh /red_standard_robot1 0.02 50
# 环境变量：
#   PITCH_STEP_RAD=0.0 QUIET=1 DURATION_SEC=0 TOPIC_SUFFIX=robot_base/gimbal_cmd

PS4='[${BASH_SOURCE}:${LINENO}] '

NS="${1:-/red_standard_robot1}"
YAW_STEP_RAD="${2:-${YAW_STEP_RAD:-0.2}}"
RATE_HZ="${3:-${RATE_HZ:-100}}"
PITCH_STEP_RAD="${PITCH_STEP_RAD:-0.0}"
QUIET="${QUIET:-1}"
DURATION_SEC="${DURATION_SEC:-0}"
TOPIC_SUFFIX="${TOPIC_SUFFIX:-robot_base/gimbal_cmd}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_d="$SCRIPT_DIR"
while [[ "$_d" != "/" && ! -f "$_d/install/setup.bash" ]]; do _d="$(dirname "$_d")"; done
WS_DIR="$_d"; unset _d

# source overlay（存在就用 overlay，否则只用 /opt/ros）
set +u
if [[ -f "$WS_DIR/install/setup.bash" ]]; then
  source "$WS_DIR/install/setup.bash"
elif [[ -f /opt/ros/humble/setup.bash ]]; then
  source /opt/ros/humble/setup.bash
fi
set -u

# 规范化 NS
if [[ -n "$NS" && "$NS" != /* ]]; then
  NS="/$NS"
fi

TOPIC="${NS}/${TOPIC_SUFFIX}"

echo "[spin_gimbal.sh] NS=${NS}"
echo "[spin_gimbal.sh] TOPIC=${TOPIC}"
echo "[spin_gimbal.sh] YAW_STEP_RAD=${YAW_STEP_RAD} rad, PITCH_STEP_RAD=${PITCH_STEP_RAD} rad"
echo "[spin_gimbal.sh] RATE_HZ=${RATE_HZ} Hz, DURATION_SEC=${DURATION_SEC} (0=forever)"

# 自检：话题存在且有订阅者（Gazebo robot_base 应该会订阅）
if ! ros2 topic list | grep -qx "${TOPIC}"; then
  echo "[spin_gimbal.sh] ERROR: topic not found: ${TOPIC}" >&2
  echo "[spin_gimbal.sh] HINT: 你可以先跑：ros2 topic list | grep gimbal_cmd" >&2
  exit 2
fi

sub_cnt="$(ros2 topic info "${TOPIC}" 2>/dev/null \
  | tr -d '\r' \
  | awk 'tolower($0) ~ /subscription count/ || $0 ~ /订阅/ {print; exit}' \
  | grep -oE '[0-9]+' \
  | head -n1 \
  || true)"
if [[ -z "${sub_cnt}" ]]; then
  echo "[spin_gimbal.sh] WARN: unable to parse subscription count for ${TOPIC}" >&2
elif [[ "${sub_cnt}" == "0" ]]; then
  echo "[spin_gimbal.sh] WARN: ${TOPIC} has 0 subscribers (Gazebo/robot_base 可能没启动)" >&2
fi

# 这条命令含义：以 RELATIVE_ANGLE 模式持续给 yaw 增量，让云台一直转。
# - yaw_type=2 / pitch_type=2 表示相对角度
# - position.yaw 每条消息增加 YAW_STEP_RAD 弧度
# - -r RATE_HZ 表示发送频率
MSG="{yaw_type: 2, pitch_type: 2, position: {yaw: ${YAW_STEP_RAD}, pitch: ${PITCH_STEP_RAD}}}"

CMD=(ros2 topic pub -r "${RATE_HZ}" "${TOPIC}" rmoss_interfaces/msg/GimbalCmd "${MSG}")

if [[ "${QUIET}" == "1" ]]; then
  if [[ "${DURATION_SEC}" == "0" ]]; then
    "${CMD[@]}" >/dev/null
  else
    timeout "${DURATION_SEC}" "${CMD[@]}" >/dev/null || true
  fi
else
  if [[ "${DURATION_SEC}" == "0" ]]; then
    "${CMD[@]}"
  else
    timeout "${DURATION_SEC}" "${CMD[@]}" || true
  fi
fi
