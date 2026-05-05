#!/bin/bash
set -euo pipefail

# Thin wrapper: delegate to Python (easier to read).
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 仿真单终端开关：true=Gazebo无头 + 建图在本终端前台；false=保持原有多终端行为。
SIM_SINGLE_TERMINAL_HEADLESS="${SIM_SINGLE_TERMINAL_HEADLESS:-true}"

# Gazebo GUI 开关：0=开启GUI窗口；1=无头模式（仅在 HEADLESS=true 时才生效）
export GAZEBO_HEADLESS="${GAZEBO_HEADLESS:-0}"

# Gazebo 启动后等待时间(秒)，等稳定后再启动 rviz2/SLAM
export GAZEBO_STARTUP_DELAY="${GAZEBO_STARTUP_DELAY:-10}"

export QT_FONT_DPI=400
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia

# Render-engine environment (ogre2 is the default; override if needed).
export IGN_GAZEBO_RENDER_ENGINE_SERVER="${IGN_GAZEBO_RENDER_ENGINE_SERVER:-ogre2}"
export IGN_GAZEBO_RENDER_ENGINE_GUI="${IGN_GAZEBO_RENDER_ENGINE_GUI:-ogre2}"

# 容器内有 xterm，允许弹多终端（覆盖 Docker 自动 no_new_terminal 逻辑）
export NO_NEW_TERMINAL="${NO_NEW_TERMINAL:-0}"

if [[ "${SIM_SINGLE_TERMINAL_HEADLESS}" == "true" ]]; then
	export NO_NEW_TERMINAL=1
fi

# Backward-compatible override name.
if [[ -n ${SLAM_PARAMS_FILE:-} ]] && [[ -z ${SIM_PARAMS_FILE:-} ]]; then
	export SIM_PARAMS_FILE="$SLAM_PARAMS_FILE"
fi

# 你想加/改 launch 参数，优先改这两行（或运行时用环境变量覆盖）。
GAZEBO_CMD=${GAZEBO_CMD:-"ros2 launch rmu_gazebo_simulator bringup_sim.launch.py"}
SLAM_CMD=${SLAM_CMD:-"ros2 launch gxu2026_nav_bringup rm_navigation_simulation_launch.py slam:=True"}
export GAZEBO_CMD SLAM_CMD

exec python3 "$SCRIPT_DIR/launch_wrapper.py" sim_mapping "$@"
