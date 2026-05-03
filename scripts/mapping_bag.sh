#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# bag 回放建图固定使用仿真时钟，并关闭自动录包，避免回放时又录一层 bag。
export REALITY_USE_SIM_TIME="True"
export AUTO_RECORD_BAG="0"

# 关闭上游驱动和 robot_state_publisher，避免与 bag 重放产生重复发布者
# /tf_static 已录制在包里，/livox/* 也由 bag 提供，下游 point_lio+slam 自行计算 /tf
export MAPPING_CMD="ros2 launch gxu2026_nav_bringup rm_navigation_reality_launch.py slam:=True use_robot_state_pub:=False use_mid360_driver:=False"

exec "$SCRIPT_DIR/mapping.sh" "$@"
