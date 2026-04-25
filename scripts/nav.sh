#!/bin/bash
set -euo pipefail

# Thin wrapper: delegate to Python (easier to read).
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 实车默认不要拉起 gnome-terminal 等图形化终端（只在当前终端运行）。
export NO_NEW_TERMINAL="${NO_NEW_TERMINAL:-1}"

export QT_FONT_DPI=192
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia

# 入口脚本默认覆盖 odin1 驱动模式：2=重定位。
export ODIN_MODE_PRESET="${ODIN_MODE_PRESET:-2}"
export NAV2_TF_WARMUP_ENABLED="${NAV2_TF_WARMUP_ENABLED:-True}"
# 重定位阶段 map->odom 可能需要较长收敛时间；默认放宽 warmup 超时，避免 Nav2 过早激活。
export NAV2_TF_WARMUP_TIMEOUT_SEC="${NAV2_TF_WARMUP_TIMEOUT_SEC:-30.0}"
export ENABLE_TIMESTAMP_MONITOR="0"
export TIMESTAMP_SYNC_MONITOR_CONFIG="${TIMESTAMP_SYNC_MONITOR_CONFIG:-$SCRIPT_DIR/config/timestamp_sync_monitor.yaml}"

# 你想加/改 launch 参数，优先改这行（或运行时用环境变量覆盖 NAVIGATION_CMD）。
NAVIGATION_CMD=${NAVIGATION_CMD:-"ros2 launch gxu2026_nav_bringup rm_navigation_reality_launch.py slam:=False use_robot_state_pub:=True nav2_tf_warmup_enabled:=${NAV2_TF_WARMUP_ENABLED} nav2_tf_warmup_timeout_sec:=${NAV2_TF_WARMUP_TIMEOUT_SEC}"}
export NAVIGATION_CMD
export KILL_EXISTING="${KILL_EXISTING:-1}"

# 透传入口脚本名，让日志按脚本入口命名。
export WRAPPER_ENTRY_NAME="$(basename "$0")"
export PYTHONPATH="$SCRIPT_DIR${PYTHONPATH:+:$PYTHONPATH}"

exec python3 -m tools.wrapper_main reality_navigation "$@"
