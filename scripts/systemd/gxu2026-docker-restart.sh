#!/usr/bin/env bash
set -euo pipefail

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

if ! command -v docker >/dev/null 2>&1; then
  echo "[INFO] docker not found; skip"
  exit 0
fi

if ! docker info >/dev/null 2>&1; then
  echo "[INFO] docker not ready; skip"
  exit 0
fi

for name in "${containers[@]}"; do
  if [[ -z "$name" ]]; then
    continue
  fi

  if docker container inspect "$name" >/dev/null 2>&1; then
    echo "[INFO] Restarting container: $name"
    docker restart "$name" >/dev/null
  else
    echo "[INFO] Skip missing container: $name"
  fi
done
