#!/usr/bin/env bash
set -euo pipefail

# 顶层入口脚本：把实现放到独立功能包 ros2_bag_tools 里
# - 优先使用 ros2 run（build/install 后更稳定）
# - 如果还没 build，也可以回退到源码脚本直接运行

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

if ! command -v ros2 >/dev/null 2>&1; then
  echo "找不到 ros2 命令。请先 source ROS2 环境 (如 /opt/ros/humble/setup.bash) 以及工作空间 install/setup.bash" >&2
  exit 1
fi

if ros2 pkg prefix ros2_bag_tools >/dev/null 2>&1; then
  exec ros2 run ros2_bag_tools record_bag "$@"
fi

SRC_SCRIPT="${WS_ROOT}/src/ros2_bag_tools/scripts/record_bag"
if [[ -x "${SRC_SCRIPT}" ]]; then
  exec "${SRC_SCRIPT}" "$@"
fi

echo "ros2_bag_tools 尚不可用：请先 colcon build 并 source install/setup.bash，或确认源码存在：${SRC_SCRIPT}" >&2
exit 1
