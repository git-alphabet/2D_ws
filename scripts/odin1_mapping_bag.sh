#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# odin1 bag 回放建图：固定使用仿真时钟，并关闭自动录包，避免回放时又录一层 bag。
# 显式传入 use_sim_time:=True，使 launch 内所有节点（robot_state_publisher、nav2 等）使用 bag 回放的 /clock，而非 wall clock
export MAPPING_CMD="ros2 launch gxu2026_nav_bringup rm_navigation_reality_launch.py slam:=True use_robot_state_pub:=True use_sim_time:=True"

exec "$SCRIPT_DIR/odin1_mapping.sh" "$@"
