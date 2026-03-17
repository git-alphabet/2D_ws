#!/usr/bin/env bash
set -e

ROS_DISTRO=${ROS_DISTRO:-humble}

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

# 让交互式 shell（Attach Shell）也能自动 source ROS 环境
# 写入 .bashrc，只写一次（在 HOME 可用时）
BASHRC_PATH="${HOME:-}/.bashrc"
if [ -n "${HOME:-}" ] && [ -d "${HOME}" ]; then
  if ! grep -q "ros/humble/setup.bash" "${BASHRC_PATH}" 2>/dev/null; then
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> "${BASHRC_PATH}"
    echo '# GXU_BRANCH_OVERLAY' >> "${BASHRC_PATH}"
    echo '_gxu_branch=default' >> "${BASHRC_PATH}"
    echo 'if [ -f /ws/.git/HEAD ]; then _gxu_head=$(</ws/.git/HEAD); [[ "$_gxu_head" == ref:\ refs/heads/* ]] && _gxu_branch="${_gxu_head#ref: refs/heads/}"; fi' >> "${BASHRC_PATH}"
    echo '_gxu_branch_safe="${_gxu_branch//[^A-Za-z0-9._-]/_}"' >> "${BASHRC_PATH}"
    echo '_gxu_cache_root=${COLCON_CACHE_ROOT:-/ws/.buildcache}' >> "${BASHRC_PATH}"
    echo '_gxu_setup="${_gxu_cache_root}/${_gxu_branch_safe}/install/setup.bash"' >> "${BASHRC_PATH}"
    echo '[ ! -f "$_gxu_setup" ] && _gxu_setup="/ws/build/.buildcache/${_gxu_branch_safe}/install/setup.bash"' >> "${BASHRC_PATH}"
    echo '[ -f "$_gxu_setup" ] && source "$_gxu_setup" || [ -f /ws/install/setup.bash ] && source /ws/install/setup.bash' >> "${BASHRC_PATH}"
  fi
else
  echo "[ros_entrypoint] HOME directory not available (${HOME:-unset}), skip .bashrc update"
fi

exec "$@"
