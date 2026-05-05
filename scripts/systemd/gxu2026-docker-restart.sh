#!/usr/bin/env bash
set -euo pipefail

GUI_DIR="${GUI_DIR:-/tmp/gxu2026-docker-gui}"
READY_FILE="${GUI_DIR}/ready"
STATUS_FILE="${GUI_DIR}/status"
STATE_DIR="/var/lib/gxu2026-docker-restart"
LAST_DISPLAY_FILE="${STATE_DIR}/last_display"
LAST_RESTART_TIME_FILE="${STATE_DIR}/last_restart_time"

COOLDOWN_SECONDS=30

if [[ ! -s "$READY_FILE" ]]; then
  echo "[INFO] GUI not ready yet; skip docker restart"
  if [[ -s "$STATUS_FILE" ]]; then
    tr '\n' ' ' < "$STATUS_FILE" | sed 's/[[:space:]]*$//'
    printf '\n'
  fi
  exit 0
fi

mkdir -p "$STATE_DIR"

current_display="$(sed -n 's/^display=//p' "$READY_FILE" | head -n 1)"
last_display="$(head -n 1 "$LAST_DISPLAY_FILE" 2>/dev/null || true)"

if [[ -z "$current_display" ]]; then
  echo "[INFO] No display selected; skip docker restart"
  exit 0
fi

if [[ "$current_display" == "$last_display" ]]; then
  echo "[INFO] Display unchanged ($current_display); skip docker restart"
  exit 0
fi

# Cooldown: 防止重启后短时间内再次重启
if [[ -f "$LAST_RESTART_TIME_FILE" ]]; then
  last_restart_time="$(head -n 1 "$LAST_RESTART_TIME_FILE" 2>/dev/null || echo 0)"
  now="$(date +%s)"
  elapsed=$(( now - last_restart_time ))
  if [[ "$elapsed" -lt "$COOLDOWN_SECONDS" ]]; then
    echo "[INFO] Cooldown active (${elapsed}s < ${COOLDOWN_SECONDS}s); skip docker restart"
    exit 0
  fi
fi

DEFAULT_CONTAINERS=(
  gxu2026-nav-robot
  gxu2026-neupan-runtime
  gxu2026-nav-laptop
)

containers=()
if [[ -n "${CONTAINER_NAMES:-}" ]]; then
  # shellcheck disable=SC2206
  containers=($CONTAINER_NAMES)
else
  containers=("${DEFAULT_CONTAINERS[@]}")
fi

if ! command -v docker >/dev/null 2>&1; then exit 0; fi
if ! docker info >/dev/null 2>&1; then exit 0; fi

echo "[INFO] Display changed: ${last_display:-none} -> ${current_display}; restarting containers"

for name in "${containers[@]}"; do
  if [[ -z "$name" ]]; then continue; fi

  if docker container inspect "$name" >/dev/null 2>&1; then
    echo "[INFO] Restarting container: $name (display=$current_display)"
    docker restart "$name" >/dev/null
  fi
done

printf '%s\n' "$current_display" > "$LAST_DISPLAY_FILE"
date +%s > "$LAST_RESTART_TIME_FILE"
