#!/bin/bash
set -euo pipefail

# One-click: start mapping on remote (headless), and ask local GUI machine to start RViz via DDS.
# Requirements:
# - On local GUI machine, run: `source /opt/ros/humble/setup.bash && source <ws>/install/setup.bash && python3 <ws>/scripts/rviz_daemon.py`
# - Remote + local must share ROS_DOMAIN_ID / RMW_IMPLEMENTATION and have ROS_LOCALHOST_ONLY=0.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export NO_NEW_TERMINAL=1
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

if [[ -n "${ROS_DOMAIN_ID:-}" ]]; then
  export ROS_DOMAIN_ID
fi
if [[ -n "${RMW_IMPLEMENTATION:-}" ]]; then
  export RMW_IMPLEMENTATION
fi

# Best-effort: trigger RViz on local machine (must have rviz_daemon running).
if command -v timeout >/dev/null 2>&1; then
  timeout 2 ros2 service call /rviz/start std_srvs/srv/Trigger "{}" >/dev/null 2>&1 || true
else
  ros2 service call /rviz/start std_srvs/srv/Trigger "{}" >/dev/null 2>&1 || true
fi

exec "${SCRIPT_DIR}/mapping_gui.sh" "$@"
