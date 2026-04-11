#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT_NAME="$(basename "$0")"

# 默认一键全停；如需改成只停 reality 或 sim，直接改这里。
TARGET="all"
CONTAINER_NAME="${DOCKER_CONTAINER:-gxu2026-nav-robot}"
WS_IN_CONTAINER="${WS_IN_CONTAINER:-/ws}"

SIM_PGID_FILE="/tmp/ros2_nav_sim.pgid"
REALITY_PGID_FILE="/tmp/ros2_nav_reality.pgid"

log() {
  echo "[$SCRIPT_NAME] $*" >&2
}

cleanup_fastdds_shm() {
  local cleaned=0
  shopt -s nullglob
  for f in /dev/shm/fastrtps_*; do
    rm -f "$f" 2>/dev/null || true
    cleaned=$((cleaned + 1))
  done
  shopt -u nullglob
  if [[ $cleaned -gt 0 ]]; then
    log "Cleaned FastDDS shm segments: $cleaned"
  fi
}

kill_by_pgid_file() {
  local pgid_file="$1"
  local title="$2"
  local pgid

  [[ -f "$pgid_file" ]] || return 0

  pgid="$(tr -d '[:space:]' < "$pgid_file" 2>/dev/null || true)"
  if [[ -z "$pgid" ]] || ! [[ "$pgid" =~ ^[0-9]+$ ]]; then
    rm -f "$pgid_file" 2>/dev/null || true
    return 0
  fi

  log "Killing $title PGID=$pgid via pgid file"
  kill -TERM "-$pgid" 2>/dev/null || true
  sleep 0.8
  kill -KILL "-$pgid" 2>/dev/null || true
  rm -f "$pgid_file" 2>/dev/null || true
}

kill_by_pattern() {
  local pattern="$1"
  local title="$2"
  local sig
  local pids
  local pid
  local pgid

  pids="$(pgrep -f "$pattern" 2>/dev/null || true)"
  [[ -n "$pids" ]] || return 0

  log "Killing existing $title pids: $(echo "$pids" | tr '\n' ' ')"

  for sig in TERM KILL; do
    while IFS= read -r pid; do
      [[ -n "$pid" ]] || continue
      if ! kill -0 "$pid" 2>/dev/null; then
        continue
      fi

      pgid="$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d '[:space:]')"
      if [[ -n "$pgid" ]] && [[ "$pgid" =~ ^[0-9]+$ ]]; then
        kill "-$sig" "-$pgid" 2>/dev/null || true
      else
        kill "-$sig" "$pid" 2>/dev/null || true
      fi
    done <<< "$pids"

    sleep 0.8
    pids="$(pgrep -f "$pattern" 2>/dev/null || true)"
    [[ -z "$pids" ]] && break
  done

  if [[ -n "$pids" ]]; then
    log "Warning: $title pids still alive: $(echo "$pids" | tr '\n' ' ')"
  fi
}

kill_sim() {
  kill_by_pgid_file "$SIM_PGID_FILE" "sim"
  cleanup_fastdds_shm
  kill_by_pattern 'bringup_sim\.launch\.py' 'bringup_sim'
  kill_by_pattern 'ruby.*ign|ign.*gazebo|gz-server|gz-gui' 'Gazebo'
  kill_by_pattern 'rm_navigation_simulation_launch\.py' 'sim nav/SLAM'
}

kill_reality() {
  kill_by_pgid_file "$REALITY_PGID_FILE" "reality"
  cleanup_fastdds_shm
  kill_by_pattern 'rm_navigation_reality_launch\.py' 'reality nav/SLAM'
  kill_by_pattern '(^|/)joint_state_publisher(\s|$)' 'joint_state_publisher'
  kill_by_pattern '(^|/)robot_state_publisher(\s|$)' 'robot_state_publisher'
  kill_by_pattern '(^|/)auto_aim_yaw_joint_state_bridge(\s|$)' 'auto_aim_yaw_bridge'
  kill_by_pattern 'component_container_isolated.*nav2_container' 'nav2_container'
}

# 容器内直接执行，容器外自动转发到目标容器。
if [[ -f "/.dockerenv" ]]; then
  if [[ "$TARGET" == "all" || "$TARGET" == "reality" ]]; then
    log "stopping reality launch group ..."
    kill_reality
  fi

  if [[ "$TARGET" == "all" || "$TARGET" == "sim" ]]; then
    log "stopping sim launch group ..."
    kill_sim
  fi

  log "stop request finished (target=$TARGET)"
  exit 0
fi

if ! command -v docker >/dev/null 2>&1; then
  echo "[stop_launch.sh] docker command not found." >&2
  exit 1
fi

if ! docker ps --format '{{.Names}}' | grep -Fxq "$CONTAINER_NAME"; then
  echo "[stop_launch.sh] container '$CONTAINER_NAME' is not running." >&2
  exit 1
fi

echo "[stop_launch.sh] stopping launch in container '$CONTAINER_NAME' (target=$TARGET) ..." >&2
exec docker exec "$CONTAINER_NAME" bash -lc "cd '$WS_IN_CONTAINER' && ./scripts/stop_launch.sh"
