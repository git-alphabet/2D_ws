#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

cd "$WS_DIR"

BRANCH_NAME="${BUILD_PROFILE:-}"
if [[ -z "$BRANCH_NAME" && -f "$WS_DIR/.git/HEAD" ]]; then
  git_head="$(<"$WS_DIR/.git/HEAD")"
  if [[ "$git_head" == ref:\ refs/heads/* ]]; then
    BRANCH_NAME="${git_head#ref: refs/heads/}"
  fi
fi
BRANCH_NAME="${BRANCH_NAME:-default}"
BRANCH_SAFE="$(echo "$BRANCH_NAME" | sed 's#[^A-Za-z0-9._-]#_#g')"

CACHE_ROOT="${COLCON_CACHE_ROOT:-$WS_DIR/.buildcache}"
mkdir -p "$CACHE_ROOT" 2>/dev/null || true
if ! (mkdir -p "$CACHE_ROOT/.perm_check_$$" 2>/dev/null && rmdir "$CACHE_ROOT/.perm_check_$$" 2>/dev/null); then
  FALLBACK_CACHE_ROOT="$WS_DIR/build/.buildcache"
  mkdir -p "$FALLBACK_CACHE_ROOT"
  CACHE_ROOT="$FALLBACK_CACHE_ROOT"
  echo "[build-profile] cache root not writable, fallback to $CACHE_ROOT"
fi
BUILD_BASE="${COLCON_BUILD_BASE:-$CACHE_ROOT/$BRANCH_SAFE/build}"
INSTALL_BASE="${COLCON_INSTALL_BASE:-$CACHE_ROOT/$BRANCH_SAFE/install}"
LOG_BASE="${COLCON_LOG_BASE:-$CACHE_ROOT/$BRANCH_SAFE/log}"

mkdir -p "$BUILD_BASE" "$INSTALL_BASE" "$LOG_BASE"
echo "[build-profile] branch=$BRANCH_NAME"
echo "[build-profile] build_base=$BUILD_BASE"
echo "[build-profile] install_base=$INSTALL_BASE"
echo "[build-profile] log_base=$LOG_BASE"

stale_link_count=0
if [[ -d "$INSTALL_BASE" ]]; then
  while IFS= read -r stale_link; do
    [[ -n "$stale_link" ]] || continue
    rm -f "$stale_link"
    stale_link_count=$((stale_link_count + 1))
  done < <(find "$INSTALL_BASE" -type l -lname '/ws/build/.buildcache/*' 2>/dev/null || true)
fi
if [[ $stale_link_count -gt 0 ]]; then
  echo "[prune] Removed $stale_link_count stale install symlink(s) from old cache root."
fi

# Cold build guard: first build on a branch can consume large memory if fully parallel.
# Auto-fallback to sequential executor unless user explicitly overrides.
COLCON_EXECUTOR_ARGS=()
if [[ "${FORCE_PARALLEL:-0}" != "1" ]]; then
  has_cache=0
  shopt -s nullglob
  cache_files=("$BUILD_BASE"/*/CMakeCache.txt)
  shopt -u nullglob
  if [[ ${#cache_files[@]} -gt 0 ]]; then
    has_cache=1
  fi
  if [[ $has_cache -eq 0 ]]; then
    COLCON_EXECUTOR_ARGS=(--executor sequential)
    echo "[build-profile] cold branch build detected, auto-fallback to sequential to reduce memory peak"
    echo "[build-profile] set FORCE_PARALLEL=1 to force parallel build"
  fi
fi

# --- Prune stale build artifacts ---
# Scan build/*/CMakeCache.txt; if the cached source dir no longer exists,
# remove that package from build/, install/, and log/ so colcon re-discovers it.
# This handles: deleted packages, directory renames, branch switches.
if [[ -d "$BUILD_BASE" ]]; then
  stale_count=0
  for cache_file in "$BUILD_BASE"/*/CMakeCache.txt; do
    [[ -f "$cache_file" ]] || continue
    pkg_build_dir="$(dirname "$cache_file")"
    pkg_name="$(basename "$pkg_build_dir")"
    cache_dir="$(grep -m1 '^CMAKE_CACHEFILE_DIR:' "$cache_file" | cut -d= -f2-)"
    if [[ -n "$cache_dir" && "$cache_dir" != "$pkg_build_dir" ]]; then
      echo "[prune] '$pkg_name': cache dir moved ('$cache_dir' -> '$pkg_build_dir'), removing stale artifacts..."
      rm -rf "$BUILD_BASE/$pkg_name"
      rm -rf "$INSTALL_BASE/$pkg_name"
      rm -rf "$LOG_BASE/latest_build/$pkg_name" 2>/dev/null || true
      stale_count=$((stale_count + 1))
      continue
    fi
    # CMAKE_HOME_DIRECTORY is the actual source directory colcon recorded
    src_dir="$(grep -m1 '^CMAKE_HOME_DIRECTORY:' "$cache_file" | cut -d= -f2-)"
    if [[ -n "$src_dir" && ! -d "$src_dir" ]]; then
      echo "[prune] '$pkg_name': cached src '$src_dir' not found, removing stale artifacts..."
      rm -rf "$BUILD_BASE/$pkg_name"
      rm -rf "$INSTALL_BASE/$pkg_name"
      rm -rf "$LOG_BASE/latest_build/$pkg_name" 2>/dev/null || true
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

# ccache 加速：仅在系统安装了 ccache 时启用，不改变原有构建流程。
CMAKE_ARGS=(-DCMAKE_BUILD_TYPE=Release)
if command -v ccache >/dev/null 2>&1; then
  export CCACHE_DIR="${CCACHE_DIR:-$CACHE_ROOT/$BRANCH_SAFE/ccache}"
  mkdir -p "$CCACHE_DIR"
  ccache --max-size "${CCACHE_MAXSIZE:-20G}" >/dev/null 2>&1 || true
  CMAKE_ARGS+=(
    -DCMAKE_C_COMPILER_LAUNCHER=ccache
    -DCMAKE_CXX_COMPILER_LAUNCHER=ccache
  )
  echo "[ccache] enabled, dir=$CCACHE_DIR"
else
  echo "[ccache] not found, build without compiler cache"
fi

# CMake 生成器选择：优先 Ninja（仅在新缓存或显式选择时启用）
BUILD_GENERATOR="${BUILD_GENERATOR:-ninja}"
CMAKE_GENERATOR=""
if [[ "$BUILD_GENERATOR" == "ninja" ]]; then
  if command -v ninja >/dev/null 2>&1; then
    CMAKE_GENERATOR="Ninja"
  else
    echo "[build-profile] ninja not found, fallback to default generator"
  fi
elif [[ "$BUILD_GENERATOR" == "default" ]]; then
  CMAKE_GENERATOR=""
else
  if command -v ninja >/dev/null 2>&1; then
    has_cmake_cache=0
    if find "$BUILD_BASE" -name CMakeCache.txt -print -quit 2>/dev/null | grep -q .; then
      has_cmake_cache=1
    fi
    if [[ $has_cmake_cache -eq 0 ]]; then
      CMAKE_GENERATOR="Ninja"
    else
      echo "[build-profile] build cache detected, keep existing generator; set BUILD_GENERATOR=ninja and clear build cache to switch"
    fi
  fi
fi
if [[ -n "$CMAKE_GENERATOR" ]]; then
  CMAKE_ARGS+=(-G "$CMAKE_GENERATOR")
  echo "[build-profile] cmake_generator=$CMAKE_GENERATOR"
else
  echo "[build-profile] cmake_generator=default"
fi

# Build the full ROS workspace in one pass.
colcon --log-base "$LOG_BASE" build \
  --build-base "$BUILD_BASE" \
  --install-base "$INSTALL_BASE" \
  "${COLCON_EXECUTOR_ARGS[@]}" \
  --symlink-install \
  --cmake-args "${CMAKE_ARGS[@]}"

# ...existing code...