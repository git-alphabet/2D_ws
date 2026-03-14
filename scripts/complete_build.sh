#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

slugify_branch_name() {
  local raw="$1"
  local slug
  slug="$(echo "$raw" | tr '[:upper:]' '[:lower:]' | sed -E 's#[^a-z0-9._-]+#_#g; s#^_+##; s#_+$##')"
  echo "${slug:-default}"
}

detect_branch_name() {
  if [[ -n "${BUILD_BRANCH:-}" ]]; then
    slugify_branch_name "$BUILD_BRANCH"
    return
  fi

  local branch
  branch="$(git -C "$WS_DIR" rev-parse --abbrev-ref HEAD 2>/dev/null || true)"
  if [[ -z "$branch" || "$branch" == "HEAD" ]]; then
    local commit
    commit="$(git -C "$WS_DIR" rev-parse --short HEAD 2>/dev/null || true)"
    branch="detached_${commit:-unknown}"
  fi
  slugify_branch_name "$branch"
}

BRANCH_NAME="$(detect_branch_name)"
BRANCH_ROOT="$WS_DIR/.build_branches/$BRANCH_NAME"
BUILD_BASE="$BRANCH_ROOT/build"
INSTALL_BASE="$BRANCH_ROOT/install"
LOG_BASE="$BRANCH_ROOT/log"

mkdir -p "$BUILD_BASE" "$INSTALL_BASE" "$LOG_BASE"

cd "$WS_DIR"

echo "[complete_build.sh] branch=$BRANCH_NAME"
echo "[complete_build.sh] build_base=$BUILD_BASE"
echo "[complete_build.sh] install_base=$INSTALL_BASE"
echo "[complete_build.sh] log_base=$LOG_BASE"

# Source ROS environment（在容器内直接执行脚本时需要）
# 临时关闭 -u，避免 ROS setup.bash 内部使用未定义变量时报错
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
if [[ -f "$INSTALL_BASE/pb_nav2_plugins/lib/liblayers.so" ]]; then
  echo "[complete_build.sh] 检测到旧版 liblayers.so，自动清理 pb_nav2_plugins 构建产物..."
  rm -rf "$BUILD_BASE/pb_nav2_plugins" "$INSTALL_BASE/pb_nav2_plugins"
fi

# Build the ROS workspace skipping NeuPAN and neupan_nav2_controller
colcon build \
  --build-base "$BUILD_BASE" \
  --install-base "$INSTALL_BASE" \
  --log-base "$LOG_BASE" \
  --executor sequential \
  --packages-skip neupan_nav2_controller \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

# Activate NeuPAN virtual environment and set PYTHONPATH
source neupan_env/bin/activate
python3 -m pip install -q "numpy<2" || true
NEUPAN_SITE_PACKAGES="neupan_env/lib/python3.10/site-packages"
if [[ -n "${PYTHONPATH:-}" ]]; then
  export PYTHONPATH="${PYTHONPATH}:${NEUPAN_SITE_PACKAGES}"
else
  export PYTHONPATH="${NEUPAN_SITE_PACKAGES}"
fi

# Build only the AI packages
colcon build \
  --build-base "$BUILD_BASE" \
  --install-base "$INSTALL_BASE" \
  --log-base "$LOG_BASE" \
  --packages-select neupan_nav2_controller \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

# Deactivate the environment
deactivate 2>/dev/null || true

# Clean PYTHONPATH
if [[ -n "${PYTHONPATH:-}" ]]; then
  PYTHONPATH="$(echo "$PYTHONPATH" | tr ':' '\n' | grep -v "neupan_env" | tr '\n' ':')"
fi