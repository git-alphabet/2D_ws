#!/usr/bin/env bash
set -e

ROS_DISTRO=${ROS_DISTRO:-humble}
GUI_ENV_SCRIPT="/ws/docker/docker_gui_env.sh"

if [ -z "${HOME:-}" ] || [ ! -d "${HOME:-}" ]; then
  export HOME="/tmp/home"
fi
mkdir -p "$HOME" 2>/dev/null || true

if [[ -f "${GUI_ENV_SCRIPT}" ]]; then
  # shellcheck disable=SC1090
  source "${GUI_ENV_SCRIPT}"
fi

if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
fi

OVERLAY_SETUP="${OVERLAY_SETUP:-}"
if [ -z "$OVERLAY_SETUP" ] && [ -n "${COLCON_INSTALL_BASE:-}" ] && [ -f "${COLCON_INSTALL_BASE}/setup.bash" ]; then
  OVERLAY_SETUP="${COLCON_INSTALL_BASE}/setup.bash"
fi
if [ -z "$OVERLAY_SETUP" ] && [ -f /ws/.git/HEAD ]; then
  GIT_HEAD="$(</ws/.git/HEAD)"
  if [[ "$GIT_HEAD" == ref:\ refs/heads/* ]]; then
    BRANCH_NAME="${GIT_HEAD#ref: refs/heads/}"
    BRANCH_SAFE="$(echo "$BRANCH_NAME" | sed 's#[^A-Za-z0-9._-]#_#g')"
    for CACHE_ROOT in "${COLCON_CACHE_ROOT:-}" "/ws/.buildcache" "/ws/build/.buildcache"; do
      [ -n "$CACHE_ROOT" ] || continue
      CANDIDATE_SETUP="${CACHE_ROOT}/${BRANCH_SAFE}/install/setup.bash"
      if [ -f "$CANDIDATE_SETUP" ]; then
        OVERLAY_SETUP="$CANDIDATE_SETUP"
        break
      fi
    done
  fi
fi
if [ -z "$OVERLAY_SETUP" ] && [ -f "/ws/install/setup.bash" ]; then
  OVERLAY_SETUP="/ws/install/setup.bash"
fi
if [ -n "$OVERLAY_SETUP" ] && [ -f "$OVERLAY_SETUP" ]; then
  # shellcheck disable=SC1090
  source "$OVERLAY_SETUP"
fi

export ROS_HOME="${ROS_HOME:-/tmp/.ros}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ros_log}"
export RCL_LOGGING_DISABLE_FILE_OUTPUT="${RCL_LOGGING_DISABLE_FILE_OUTPUT:-1}"
mkdir -p "$ROS_HOME" "$ROS_LOG_DIR" 2>/dev/null || true

if [ ! -w "$ROS_LOG_DIR" ]; then
  export ROS_LOG_DIR="/tmp/ros_log"
  mkdir -p "$ROS_LOG_DIR" 2>/dev/null || true
fi

# 让交互式 shell（Attach Shell）也能自动 source ROS 环境
# 写入 .bashrc，只写一次（在 HOME 可用时）
BASHRC_PATH="${HOME:-}/.bashrc"
if [ -n "${HOME:-}" ] && [ -d "${HOME}" ]; then
  if grep -q '\[ -f "\$_gxu_setup" \] && source "\$_gxu_setup" || \[ -f /ws/install/setup.bash \] && source /ws/install/setup.bash' "${BASHRC_PATH}" 2>/dev/null; then
    sed -i 's#\[ -f "\$_gxu_setup" \] && source "\$_gxu_setup" || \[ -f /ws/install/setup.bash \] && source /ws/install/setup.bash#if [ -f "$_gxu_setup" ]; then source "$_gxu_setup"; elif [ -f /ws/install/setup.bash ]; then source /ws/install/setup.bash; fi#' "${BASHRC_PATH}"
  fi

  if ! grep -q "ros/humble/setup.bash" "${BASHRC_PATH}" 2>/dev/null; then
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> "${BASHRC_PATH}"
    echo '# GXU_BRANCH_OVERLAY' >> "${BASHRC_PATH}"
    echo '_gxu_branch=default' >> "${BASHRC_PATH}"
    echo 'if [ -f /ws/.git/HEAD ]; then _gxu_head=$(</ws/.git/HEAD); [[ "$_gxu_head" == ref:\ refs/heads/* ]] && _gxu_branch="${_gxu_head#ref: refs/heads/}"; fi' >> "${BASHRC_PATH}"
    echo '_gxu_branch_safe="${_gxu_branch//[^A-Za-z0-9._-]/_}"' >> "${BASHRC_PATH}"
    echo '_gxu_cache_root=${COLCON_CACHE_ROOT:-/ws/.buildcache}' >> "${BASHRC_PATH}"
    echo '_gxu_setup="${_gxu_cache_root}/${_gxu_branch_safe}/install/setup.bash"' >> "${BASHRC_PATH}"
    echo '[ ! -f "$_gxu_setup" ] && _gxu_setup="/ws/build/.buildcache/${_gxu_branch_safe}/install/setup.bash"' >> "${BASHRC_PATH}"
    echo 'if [ -f "$_gxu_setup" ]; then source "$_gxu_setup"; elif [ -f /ws/install/setup.bash ]; then source /ws/install/setup.bash; fi' >> "${BASHRC_PATH}"
  fi
  if ! grep -q "GXU_ROS_LOGGING_ENV" "${BASHRC_PATH}" 2>/dev/null; then
    echo '# GXU_ROS_LOGGING_ENV' >> "${BASHRC_PATH}"
    echo 'export ROS_HOME=${ROS_HOME:-/tmp/.ros}' >> "${BASHRC_PATH}"
    echo 'export ROS_LOG_DIR=${ROS_LOG_DIR:-/tmp/ros_log}' >> "${BASHRC_PATH}"
    echo 'export RCL_LOGGING_DISABLE_FILE_OUTPUT=${RCL_LOGGING_DISABLE_FILE_OUTPUT:-1}' >> "${BASHRC_PATH}"
    echo 'mkdir -p "$ROS_HOME" "$ROS_LOG_DIR" 2>/dev/null || true' >> "${BASHRC_PATH}"
  fi

  # GUI 环境可能在 NoMachine 启动后变化，确保新 shell 自动刷新 DISPLAY/XAUTHORITY
  if ! grep -q "GXU_DOCKER_GUI_ENV" "${BASHRC_PATH}" 2>/dev/null; then
    echo '# GXU_DOCKER_GUI_ENV' >> "${BASHRC_PATH}"
    echo 'if [ -f /ws/docker/docker_gui_env.sh ]; then source /ws/docker/docker_gui_env.sh; fi' >> "${BASHRC_PATH}"
  fi

  # 兼容 docker exec 常见的 `bash -lc`：登录 shell 会读取 .bash_profile，默认不会读取 .bashrc。
  BASH_PROFILE_PATH="${HOME:-}/.bash_profile"
  if ! grep -q "GXU_BASH_PROFILE_LOAD_BASHRC" "${BASH_PROFILE_PATH}" 2>/dev/null; then
    echo '# GXU_BASH_PROFILE_LOAD_BASHRC' >> "${BASH_PROFILE_PATH}"
    echo 'if [ -f "$HOME/.bashrc" ]; then source "$HOME/.bashrc"; fi' >> "${BASH_PROFILE_PATH}"
  fi
else
  echo "[ros_entrypoint] HOME directory not available (${HOME:-unset}), skip .bashrc update"
fi

exec "$@"
