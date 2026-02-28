#!/bin/bash
set -euo pipefail

# Thin wrapper: delegate to Python (easier to read).
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 实车默认不要拉起 gnome-terminal 等图形化终端（只在当前终端运行）。
export NO_NEW_TERMINAL="${NO_NEW_TERMINAL:-1}"

export QT_FONT_DPI=192
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia

# 你想加/改 launch 参数，优先改这行（或运行时用环境变量覆盖 NAVIGATION_CMD）。
NAVIGATION_CMD=${NAVIGATION_CMD:-"ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py slam:=False use_robot_state_pub:=True"}
export NAVIGATION_CMD

exec python3 "$SCRIPT_DIR/launch_wrapper.py" reality_navigation "$@"
