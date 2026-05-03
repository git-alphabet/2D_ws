#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# bag 回放建图固定使用仿真时钟，并关闭自动录包，避免回放时又录一层 bag。
export REALITY_USE_SIM_TIME="True"
export AUTO_RECORD_BAG="0"

# 关闭上游 mid360_driver，避免与 bag 重放的 /livox/* 产生重复发布者
# robot_state_publisher 必须开启补全 bag 中可能缺失的 /tf_static（如 base_footprint 链路）
# 实车录制时未启用完整的 /tf_static，回放依赖 robot_state_publisher 重建 TF 树
export MAPPING_CMD="ros2 launch gxu2026_nav_bringup rm_navigation_reality_launch.py slam:=True use_robot_state_pub:=True use_mid360_driver:=False"

exec "$SCRIPT_DIR/mapping.sh" "$@"
