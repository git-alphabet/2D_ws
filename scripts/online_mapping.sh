#!/bin/bash
set -euo pipefail

# One-click: start mapping on remote (headless).
# 默认不启动 RViz/不拉起图形化终端；可通过 Foxglove Bridge 在本地 Foxglove Studio 连接查看。

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export NO_NEW_TERMINAL=1
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

if [[ -n "${ROS_DOMAIN_ID:-}" ]]; then
  export ROS_DOMAIN_ID
fi
if [[ -n "${RMW_IMPLEMENTATION:-}" ]]; then
  export RMW_IMPLEMENTATION
fi

exec "${SCRIPT_DIR}/mapping.sh" "$@"
