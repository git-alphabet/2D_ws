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

# 重定位导航入口：显式使用 odin_map_mode:=2，并在 nav2 侧开启 slam_toolbox 建图。
NAVIGATION_CMD=${NAVIGATION_CMD:-"ros2 launch gxu2026_nav_bringup rm_navigation_reality_launch.py slam:=True use_robot_state_pub:=True odin_map_mode:=2 nav2_tf_warmup_enabled:=False"}
export NAVIGATION_CMD
export KILL_EXISTING="${KILL_EXISTING:-1}"

# 透传入口脚本名，让日志按脚本入口命名。
export WRAPPER_ENTRY_NAME="$(basename "$0")"
export PYTHONPATH="$SCRIPT_DIR${PYTHONPATH:+:$PYTHONPATH}"

exec python3 -m tools.wrapper_main reality_navigation "$@"
