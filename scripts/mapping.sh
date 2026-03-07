#!/bin/bash
set -euo pipefail

# Thin wrapper: delegate to Python (easier to read).
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 实车默认不要拉起 gnome-terminal 等图形化终端（只在当前终端运行）。
export NO_NEW_TERMINAL="${NO_NEW_TERMINAL:-1}"

export QT_FONT_DPI=192
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia

export START_RVIZ="${START_RVIZ:-0}"
export KILL_EXISTING="${KILL_EXISTING:-1}"

# 你想加/改 launch 参数，优先改这行（或运行时用环境变量覆盖 MAPPING_CMD）。
export MAPPING_CMD=${MAPPING_CMD:-"ros2 launch gxu2026_nav_bringup rm_navigation_reality_launch.py slam:=True use_robot_state_pub:=True"}
export MAPPING_CMD

# 清理 FastDDS 残留的共享内存段，防止重复启动时 DDS 初始化卡死
rm -f /dev/shm/fastrtps_* 2>/dev/null || true
# 杀掉可能残留的 nav2 component container（防止节点名冲突导致 LoadComposableNodes 卡死）
pkill -f 'component_container_isolated.*nav2_container' 2>/dev/null || true
sleep 0.3

exec python3 "$SCRIPT_DIR/launch_wrapper.py" reality_mapping "$@"
