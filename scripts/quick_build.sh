#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

cd "$WS_DIR"

# Source ROS environment（在容器内直接执行脚本时需要）
ROS_DISTRO="${ROS_DISTRO:-humble}"
set +u
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

# 清理环境里已失效的前缀路径（例如包重命名后残留的 /ws/install/xxx）
sanitize_prefix_path_var() {
  local var_name="$1"
  local value="${!var_name:-}"
  local cleaned=""

  if [[ -z "$value" ]]; then
    return
  fi

  IFS=':' read -r -a parts <<< "$value"
  for p in "${parts[@]}"; do
    [[ -z "$p" ]] && continue
    if [[ -d "$p" ]]; then
      if [[ -n "$cleaned" ]]; then
        cleaned+="${cleaned:+:}$p"
      else
        cleaned="$p"
      fi
    fi
  done

  export "$var_name=$cleaned"
}

sanitize_prefix_path_var AMENT_PREFIX_PATH
sanitize_prefix_path_var CMAKE_PREFIX_PATH
sanitize_prefix_path_var COLCON_PREFIX_PATH

# Build the ROS workspace skipping NeuPAN and neupan_nav2_controller
colcon build  --packages-skip neupan_nav2_controller --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Activate NeuPAN virtual environment and set PYTHONPATH
source neupan_env/bin/activate
NEUPAN_SITE_PACKAGES="neupan_env/lib/python3.10/site-packages"
if [[ -n "${PYTHONPATH:-}" ]]; then
  export PYTHONPATH="${PYTHONPATH}:${NEUPAN_SITE_PACKAGES}"
else
  export PYTHONPATH="${NEUPAN_SITE_PACKAGES}"
fi

# Build only the AI packages
colcon build \
  --packages-select neupan_nav2_controller \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

# Deactivate the environment
deactivate 2>/dev/null || true

# Clean PYTHONPATH
if [[ -n "${PYTHONPATH:-}" ]]; then
  PYTHONPATH="$(echo "$PYTHONPATH" | tr ':' '\n' | grep -v "neupan_env" | tr '\n' ':')"
fi