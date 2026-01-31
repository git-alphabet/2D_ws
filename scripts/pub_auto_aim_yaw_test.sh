#!/usr/bin/env bash
set -e

# 用法：
#   ./scripts/pub_auto_aim_yaw_test.sh                  # 发布到 /auto_aim_yaw
#   ./scripts/pub_auto_aim_yaw_test.sh /red_standard_robot1  # 发布到 /red_standard_robot1/auto_aim_yaw

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
NS="${1:-}"

# ROS 的 setup 脚本里会读取未定义变量（如 AMENT_TRACE_SETUP_FILES）。
# 如果开启 nounset（set -u）会直接报错退出，所以这里临时关闭。
set +u
source /opt/ros/humble/setup.bash
source "$WS_ROOT/install/local_setup.bash"
set -u

# 关键：让 ros2 CLI 能解析到 sp_msgs/msg/Float32Stamped
export AMENT_PREFIX_PATH="$WS_ROOT/install/sp_msgs:${AMENT_PREFIX_PATH:-}"

TOPIC="/auto_aim_yaw"
if [[ -n "$NS" ]]; then
  TOPIC="$NS$TOPIC"
fi

echo "Publishing sp_msgs/msg/Float32Stamped -> $TOPIC"
exec ros2 topic pub --rate 10 "$TOPIC" sp_msgs/msg/Float32Stamped "{data: 1.23}"
