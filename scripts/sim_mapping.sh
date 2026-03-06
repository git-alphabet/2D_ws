#!/bin/bash
set -euo pipefail

# Thin wrapper: delegate to Python (easier to read).
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export QT_FONT_DPI=192
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia

# Render-engine environment (ogre2 is the default; override if needed).
export IGN_GAZEBO_RENDER_ENGINE_SERVER="${IGN_GAZEBO_RENDER_ENGINE_SERVER:-ogre2}"
export IGN_GAZEBO_RENDER_ENGINE_GUI="${IGN_GAZEBO_RENDER_ENGINE_GUI:-ogre2}"

# 容器内有 xterm，允许弹多终端（覆盖 Docker 自动 no_new_terminal 逻辑）
export NO_NEW_TERMINAL="${NO_NEW_TERMINAL:-0}"

# Backward-compatible override name.
if [[ -n ${SLAM_PARAMS_FILE:-} ]] && [[ -z ${SIM_PARAMS_FILE:-} ]]; then
	export SIM_PARAMS_FILE="$SLAM_PARAMS_FILE"
fi

# 你想加/改 launch 参数，优先改这两行（或运行时用环境变量覆盖）。
GAZEBO_CMD=${GAZEBO_CMD:-"ros2 launch rmu_gazebo_simulator bringup_sim.launch.py"}
SLAM_CMD=${SLAM_CMD:-"ros2 launch gxu2026_nav_bringup rm_navigation_simulation_launch.py slam:=True"}
export GAZEBO_CMD SLAM_CMD

exec python3 "$SCRIPT_DIR/launch_wrapper.py" sim_mapping "$@"
