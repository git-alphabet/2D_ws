#!/bin/bash
set -euo pipefail

# Thin wrapper: delegate to Python (easier to read).
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export QT_FONT_DPI=192
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia

export RCUTILS_LOGGING_SEVERITY=${RCUTILS_LOGGING_SEVERITY:-INFO}

# Backward-compatible override name.
if [[ -n ${NAV_PARAMS_FILE:-} ]] && [[ -z ${SIM_PARAMS_FILE:-} ]]; then
	export SIM_PARAMS_FILE="$NAV_PARAMS_FILE"
fi

# 你想加/改 launch 参数，优先改这两行（或运行时用环境变量覆盖）。
GAZEBO_CMD=${GAZEBO_CMD:-"ros2 launch rmu_gazebo_simulator bringup_sim.launch.py"}
NAV_CMD=${NAV_CMD:-"ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py world:=rmuc_2025 slam:=False"}
export GAZEBO_CMD NAV_CMD

exec python3 "$SCRIPT_DIR/launch_wrapper.py" sim_nav "$@"
