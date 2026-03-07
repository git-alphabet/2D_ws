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

# Odin1 驱动配置（参考 odin_ros_driver README）
ODIN_CONFIG_FILE="${ODIN_CONFIG_FILE:-$SCRIPT_DIR/../src/odin_ros_driver/config/control_command.yaml}"
if [[ -f "$ODIN_CONFIG_FILE" ]]; then
	mode=$(sed -n 's/^[[:space:]]*custom_map_mode:[[:space:]]*\([0-9]\+\).*$/\1/p' "$ODIN_CONFIG_FILE" | head -n1 || true)
	if [[ -n "$mode" && "$mode" != "1" ]]; then
		echo "[mapping.sh] 警告：检测到 custom_map_mode=$mode（建议建图时为 1: SLAM mode）" >&2
		echo "[mapping.sh] 配置文件：$ODIN_CONFIG_FILE" >&2
	fi
fi

# 你想加/改 launch 参数，优先改这行（或运行时用环境变量覆盖 MAPPING_CMD）。
# odin_map_mode:=1  → odin SLAM 模式，自己发布 map->odom TF + 保存 .bin
# publish_static_map_tf:=False → 禁用 static identity TF，避免与 odin 冲突
# slam:=True → 启动 slam_toolbox 同步保存 pgm 地图
MAPPING_CMD=${MAPPING_CMD:-"ros2 launch gxu2026_nav_bringup rm_navigation_reality_launch.py slam:=True use_robot_state_pub:=True odin_map_mode:=1 publish_static_map_tf:=False"}
export MAPPING_CMD

exec python3 "$SCRIPT_DIR/launch_wrapper.py" reality_mapping "$@"
