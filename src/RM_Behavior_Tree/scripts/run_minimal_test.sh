#!/usr/bin/env bash
set -euo pipefail

# Minimal test runner for DetectRespawnAndSetRecovery
# Usage: bash run_minimal_test.sh

WS_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
LOGDIR=/tmp

echo "Sourcing ROS and workspace setups..."
if [ -f /opt/ros/humble/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi
if [ -f "$WS_ROOT/install/local_setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$WS_ROOT/install/local_setup.bash"
elif [ -f "$WS_ROOT/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$WS_ROOT/install/setup.bash"
fi

echo "Killing previous test processes (if any)"
pkill -f robot_status_publisher || true
pkill -f detect_respawn_checker || true
pkill -f rm_behavior_tree_minimal || true
sleep 0.3

run_or_fallback() {
  pkg="$1"; exe="$2"; logfile="$3"
  echo "Starting $pkg:$exe -> $logfile"
  if command -v ros2 >/dev/null 2>&1; then
    ros2 run "$pkg" "$exe" >"$logfile" 2>&1 &
  else
    if [ -x "$WS_ROOT/install/bin/$exe" ]; then
      "$WS_ROOT/install/bin/$exe" >"$logfile" 2>&1 &
    else
      echo "Could not start $exe (no ros2 and no $WS_ROOT/install/bin/$exe)" >&2
      return 1
    fi
  fi
  echo $! >"${logfile}.pid"
}

echo "Starting publisher..."
run_or_fallback rm_bt_test_publishers robot_status_publisher "$LOGDIR/rm_pub.log"
sleep 0.6

echo "Starting checker..."
run_or_fallback rm_bt_test_publishers detect_respawn_checker "$LOGDIR/rm_checker.log"
sleep 0.6

echo "Starting BT minimal runner..."
run_or_fallback rm_behavior_tree rm_behavior_tree_minimal "$LOGDIR/rm_bt.log"

echo "Started. Logs:"
echo "  Publisher: $LOGDIR/rm_pub.log"
echo "  Checker:   $LOGDIR/rm_checker.log"
echo "  BT:        $LOGDIR/rm_bt.log"

echo "To watch logs: tail -f $LOGDIR/rm_pub.log $LOGDIR/rm_checker.log $LOGDIR/rm_bt.log"
