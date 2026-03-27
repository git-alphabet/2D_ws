#!/usr/bin/env bash
set -euo pipefail

MODE="${NAV_MODE:-navigation}"

export NO_NEW_TERMINAL="${NO_NEW_TERMINAL:-1}"
export KILL_EXISTING="${KILL_EXISTING:-1}"
export START_RVIZ="${START_RVIZ:-1}"
export AUTO_RECORD_BAG="${AUTO_RECORD_BAG:-1}"
export AUTO_RECORD_BAG_MODE="${AUTO_RECORD_BAG_MODE:-full}"

case "$MODE" in
  mapping)
    exec /ws/scripts/mapping.sh
    ;;
  navigation)
    exec /ws/scripts/start_navigation.sh
    ;;
  *)
    echo "[nav_mode_launcher] ERROR: NAV_MODE must be 'mapping' or 'navigation', got '$MODE'" >&2
    exit 2
    ;;
esac