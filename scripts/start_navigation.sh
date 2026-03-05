#!/bin/bash
set -euo pipefail

# Thin wrapper: delegate to Python (easier to read).
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 实车默认不要拉起 gnome-terminal 等图形化终端（只在当前终端运行）。
export NO_NEW_TERMINAL="${NO_NEW_TERMINAL:-1}"

export QT_FONT_DPI=192
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia

# Odin1 驱动配置（参考 odin_ros_driver README）
ODIN_CONFIG_FILE="${ODIN_CONFIG_FILE:-$SCRIPT_DIR/../src/odin_ros_driver/config/control_command.yaml}"
if [[ -f "$ODIN_CONFIG_FILE" ]]; then
	mode=$(sed -n 's/^[[:space:]]*custom_map_mode:[[:space:]]*\([0-9]\+\).*$/\1/p' "$ODIN_CONFIG_FILE" | head -n1 || true)
	if [[ "$mode" == "1" ]]; then
		echo "[start_navigation.sh] 警告：检测到 custom_map_mode=1（SLAM建图模式）。" >&2
		echo "[start_navigation.sh] 若为实车导航，建议改为 0(odometry) 或 2(relocalization)。" >&2
		echo "[start_navigation.sh] 配置文件：$ODIN_CONFIG_FILE" >&2
	fi
fi

# 你想加/改 launch 参数，优先改这行（或运行时用环境变量覆盖 NAVIGATION_CMD）。
NAVIGATION_CMD=${NAVIGATION_CMD:-"ros2 launch gxu2026_nav_bringup rm_navigation_reality_launch.py slam:=False use_robot_state_pub:=True"}
export NAVIGATION_CMD

exec python3 "$SCRIPT_DIR/launch_wrapper.py" reality_navigation "$@"
