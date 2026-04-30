#!/usr/bin/env bash
set -euo pipefail

GUI_DIR="${GUI_DIR:-/tmp/gxu2026-docker-gui}"
READY_FILE="${GUI_DIR}/ready"
SESSION_FINGERPRINT_FILE="${GUI_DIR}/session_fingerprint"
STATUS_FILE="${GUI_DIR}/status"
STATE_DIR="/var/lib/gxu2026-docker-restart"
LAST_RESTART_FILE="${STATE_DIR}/last_session_fingerprint"

if [[ ! -s "$READY_FILE" || ! -s "$SESSION_FINGERPRINT_FILE" ]]; then
  echo "[INFO] GUI not ready yet; skip docker restart"
  if [[ -s "$STATUS_FILE" ]]; then
    tr '\n' ' ' < "$STATUS_FILE" | sed 's/[[:space:]]*$//'
    printf '\n'
  fi
  exit 0
fi

mkdir -p "$STATE_DIR"

current_fingerprint="$(head -n 1 "$SESSION_FINGERPRINT_FILE" 2>/dev/null || true)"
last_fingerprint="$(head -n 1 "$LAST_RESTART_FILE" 2>/dev/null || true)"
selected_display="$(sed -n 's/^display=//p' "$READY_FILE" | head -n 1)"

if [[ -z "$current_fingerprint" ]]; then
  echo "[INFO] GUI fingerprint is empty; skip docker restart"
  exit 0
fi

if [[ "$current_fingerprint" == "$last_fingerprint" ]]; then
  echo "[INFO] GUI session already handled for ${selected_display:-unknown display}; skip docker restart"
  exit 0
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

for name in "${containers[@]}"; do
  if [[ -z "$name" ]]; then continue; fi

  if docker container inspect "$name" >/dev/null 2>&1; then
    echo "[INFO] Restarting docker UI container on ${selected_display:-unknown display}: $name"
    docker restart "$name" >/dev/null
  fi
done

printf '%s\n' "$current_fingerprint" > "$LAST_RESTART_FILE"
