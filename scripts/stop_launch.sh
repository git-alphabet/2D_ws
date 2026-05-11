#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT_NAME="$(basename "$0")"

# 默认一键全停；如需改成只停 reality 或 sim，直接改这里。
TARGET="all"
WS_IN_CONTAINER="${WS_IN_CONTAINER:-/ws}"
# 容器外执行时自动检测这些容器（可用 DOCKER_CONTAINERS 覆盖，空格或逗号分隔）。
CONTAINER_CANDIDATES="${DOCKER_CONTAINERS:-gxu2026-nav-laptop gxu2026-nav-robot gxu2026-neupan-runtime}"
ODIN_SHUTDOWN_TIMEOUT="${ODIN_SHUTDOWN_TIMEOUT:-30}"

SIM_PGID_FILE="/tmp/ros2_nav_sim.pgid"
REALITY_PGID_FILE="/tmp/ros2_nav_reality.pgid"

log() {
  echo "[$SCRIPT_NAME] $*" >&2
}

# 检测建图相关进程是否存在（wrapper_main.py 是主进程，它有 pre_shutdown_hook）
has_mapping_processes() {
  local pids
  # 查找 wrapper_main.py 的建图模式进程
  pids="$(pgrep -f 'wrapper_main.*reality_mapping|rm_navigation_reality_launch\.py.*slam:=True' 2>/dev/null || true)"
  [[ -n "$pids" ]]
}

# 触发建图程序的自动保存（发送 SIGINT 给 wrapper_main.py 让其 pre_shutdown_hook 执行）
# wrapper_main.py 收到 SIGINT 后会依次执行：save_odin_bin -> save_map_2d -> sleep(grace)
# 必须等它自然退出，否则 kill_reality 会打断正在进行的地图保存。
trigger_mapping_save() {
  local pid

  if ! has_mapping_processes; then
    log "no mapping processes found, skip auto-save"
    return 0
  fi

  pid="$(pgrep -f 'wrapper_main.*reality_mapping' 2>/dev/null | head -1 || true)"
  if [[ -z "$pid" ]]; then
    log "wrapper_main mapping process not found, skip auto-save"
    return 0
  fi

  log "found wrapper_main mapping (PID=$pid), sending SIGINT to trigger auto-save ..."

  # 只发送 SIGINT 给 wrapper_main.py 主进程本身
  kill -INT "$pid" 2>/dev/null || true

  # 等待 wrapper_main.py 自然退出（它内部会完成所有保存操作）
  log "waiting for wrapper_main (PID=$pid) to finish map auto-save ..."
  local wait_count=0
  local max_wait=120
  while kill -0 "$pid" 2>/dev/null; do
    sleep 1
    wait_count=$((wait_count + 1))
    if [[ $wait_count -ge $max_wait ]]; then
      log "timeout (${max_wait}s) waiting for wrapper_main, force continue"
      break
    fi
  done
  log "wrapper_main finished (waited ${wait_count}s)"
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
  local force_kill="${3:-1}"
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
  if [[ "$force_kill" -eq 1 ]]; then
    kill -KILL "-$pgid" 2>/dev/null || true
  fi
  rm -f "$pgid_file" 2>/dev/null || true
}

stop_odin_driver() {
  local pattern='(^|/)host_sdk_sample(\s|$)'
  local pids
  local start_ts
  local warned=0

  pids="$(pgrep -f "$pattern" 2>/dev/null || true)"
  [[ -n "$pids" ]] || return 0

  log "Gracefully shutting down odin driver pids: $(echo "$pids" | tr '\n' ' ')"
  while IFS= read -r pid; do
    [[ -n "$pid" ]] || continue
    kill -TERM "$pid" 2>/dev/null || true
  done <<< "$pids"

  start_ts="$(date +%s)"
  while pgrep -f "$pattern" >/dev/null 2>&1; do
    sleep 1
    if [[ "$ODIN_SHUTDOWN_TIMEOUT" -gt 0 ]] && [[ $warned -eq 0 ]]; then
      local now_ts
      now_ts="$(date +%s)"
      if (( now_ts - start_ts >= ODIN_SHUTDOWN_TIMEOUT )); then
        log "Odin driver still alive after ${ODIN_SHUTDOWN_TIMEOUT}s, keep waiting..."
        warned=1
      fi
    fi
  done
  log "Odin driver exited."
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
  kill_by_pgid_file "$REALITY_PGID_FILE" "reality" 0
  cleanup_fastdds_shm
  stop_odin_driver
  kill_by_pattern 'rm_navigation_reality_launch\.py' 'reality nav/SLAM'
  kill_by_pattern '(^|/)mid360_driver_node(\s|$)' 'mid360_driver'
  kill_by_pattern '(^|/)pointlio_mapping(\s|$)' 'pointlio_mapping'
  kill_by_pattern '(^|/)joint_state_publisher(\s|$)' 'joint_state_publisher'
  kill_by_pattern '(^|/)robot_state_publisher(\s|$)' 'robot_state_publisher'
  kill_by_pattern '(^|/)auto_aim_yaw_joint_state_bridge(\s|$)' 'auto_aim_yaw_bridge'
  kill_by_pattern 'component_container_isolated.*nav2_container' 'nav2_container'
}

stop_in_container() {
  local container_name="$1"

  log "stopping launch in container '$container_name' (target=$TARGET) ..."
  if ! docker exec "$container_name" bash -lc "
set -euo pipefail

if [[ -f '$WS_IN_CONTAINER/scripts/stop_launch.sh' ]]; then
  cd '$WS_IN_CONTAINER'
  bash ./scripts/stop_launch.sh
  exit 0
fi

echo '[stop_launch.sh] stop script not found in container, run fallback kill patterns.' >&2
if command -v pkill >/dev/null 2>&1; then
  pkill -f 'rm_navigation_(reality|simulation)_launch\\.py' 2>/dev/null || true
  pkill -f 'bringup_sim\\.launch\\.py' 2>/dev/null || true
  pkill -f 'component_container_isolated.*nav2_container' 2>/dev/null || true
fi
"; then
    log "Warning: stop command in container '$container_name' exited non-zero; continue."
  fi
}

# 容器内直接执行，容器外自动转发到目标容器。
if [[ -f "/.dockerenv" ]]; then
  if [[ "$TARGET" == "all" || "$TARGET" == "reality" ]]; then
    # 先触发建图程序的自动保存
    trigger_mapping_save
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

running_containers="$(docker ps --format '{{.Names}}' 2>/dev/null || true)"
if [[ -z "$running_containers" ]]; then
  echo "[stop_launch.sh] no running containers." >&2
  exit 1
fi

matched_containers=()
for container_name in ${CONTAINER_CANDIDATES//,/ }; do
  [[ -n "$container_name" ]] || continue
  if grep -Fxq "$container_name" <<< "$running_containers"; then
    matched_containers+=("$container_name")
  fi
done

if [[ ${#matched_containers[@]} -eq 0 ]]; then
  echo "[stop_launch.sh] none of target containers are running: ${CONTAINER_CANDIDATES}" >&2
  exit 1
fi

for container_name in "${matched_containers[@]}"; do
  stop_in_container "$container_name"
done

log "stop request finished in ${#matched_containers[@]} container(s)."
