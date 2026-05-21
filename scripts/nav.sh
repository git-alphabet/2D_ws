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
export NAV2_TF_WARMUP_ENABLED="${NAV2_TF_WARMUP_ENABLED:-False}"
# Odin mode2 may internally fall back to SLAM before it publishes map->odom.
export NAV2_TF_WARMUP_TIMEOUT_SEC="${NAV2_TF_WARMUP_TIMEOUT_SEC:-30.0}"
export ENABLE_TIMESTAMP_MONITOR="0"
export TIMESTAMP_SYNC_MONITOR_CONFIG="${TIMESTAMP_SYNC_MONITOR_CONFIG:-$SCRIPT_DIR/config/timestamp_sync_monitor.yaml}"

# Startup gate: ensure TF is ready before launching navigation
export NAV2_STARTUP_GATE_ENABLED="${NAV2_STARTUP_GATE_ENABLED:-true}"
export NAV2_TF_WARMUP_TARGET_FRAME="${NAV2_TF_WARMUP_TARGET_FRAME:-map}"
export NAV2_TF_WARMUP_SOURCE_FRAME="${NAV2_TF_WARMUP_SOURCE_FRAME:-gimbal_yaw_fake}"

# 你想加/改 launch 参数，优先改这行（或运行时用环境变量覆盖 NAVIGATION_CMD）。
# Keep Nav2 in localization/navigation mode so the saved 2D map is loaded.
NAVIGATION_CMD=${NAVIGATION_CMD:-"ros2 launch gxu2026_nav_bringup rm_navigation_reality_launch.py slam:=False use_robot_state_pub:=True odin_map_mode:=2 nav2_tf_warmup_enabled:=${NAV2_TF_WARMUP_ENABLED} nav2_tf_warmup_timeout_sec:=${NAV2_TF_WARMUP_TIMEOUT_SEC}"}
export NAVIGATION_CMD
export KILL_EXISTING="${KILL_EXISTING:-1}"

# 透传入口脚本名，让日志按脚本入口命名。
export WRAPPER_ENTRY_NAME="$(basename "$0")"
export PYTHONPATH="$SCRIPT_DIR${PYTHONPATH:+:$PYTHONPATH}"

exec python3 -m tools.wrapper_main reality_navigation "$@"
