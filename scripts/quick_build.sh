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