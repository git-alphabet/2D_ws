#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

cd "$WS_DIR"

# --- Prune stale build artifacts ---
# Scan build/*/CMakeCache.txt; if the cached source dir no longer exists,
# remove that package from build/, install/, and log/ so colcon re-discovers it.
# This handles: deleted packages, directory renames, branch switches.
if [[ -d build ]]; then
  stale_count=0
  for cache_file in build/*/CMakeCache.txt; do
    [[ -f "$cache_file" ]] || continue
    pkg_build_dir="$(dirname "$cache_file")"
    pkg_name="$(basename "$pkg_build_dir")"
    # CMAKE_HOME_DIRECTORY is the actual source directory colcon recorded
    src_dir="$(grep -m1 '^CMAKE_HOME_DIRECTORY:' "$cache_file" | cut -d= -f2-)"
    if [[ -n "$src_dir" && ! -d "$src_dir" ]]; then
      echo "[prune] '$pkg_name': cached src '$src_dir' not found, removing stale artifacts..."
      rm -rf "build/$pkg_name"
      rm -rf "install/$pkg_name"
      rm -rf "log/latest_build/$pkg_name" 2>/dev/null || true
      stale_count=$((stale_count + 1))
    fi
  done
  if [[ $stale_count -gt 0 ]]; then
    echo "[prune] Removed $stale_count stale package(s)."
  else
    echo "[prune] No stale build artifacts found."
  fi
fi

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

# 清理 pb_nav2_plugins 旧产物，防止切换分支/库名变更后残留 .so 导致 dlopen 符号缺失
# （libpb_layers.so 曾命名为 liblayers.so，旧文件若留在 install 会造成 undefined symbol）
if [[ -f "$WS_DIR/install/pb_nav2_plugins/lib/liblayers.so" ]]; then
  echo "[quick_build.sh] 检测到旧版 liblayers.so，自动清理 pb_nav2_plugins 构建产物..."
  rm -rf "$WS_DIR/build/pb_nav2_plugins" "$WS_DIR/install/pb_nav2_plugins"
fi

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