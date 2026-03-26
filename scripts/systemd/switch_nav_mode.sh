#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "Usage: $0 <navigation|mapping>" >&2
  exit 2
fi

MODE="$1"
if [[ "$MODE" != "navigation" && "$MODE" != "mapping" ]]; then
  echo "ERROR: mode must be navigation or mapping" >&2
  exit 2
fi

ENV_FILE="$HOME/.config/gxu2026/nav-stack.env"
if [[ ! -f "$ENV_FILE" ]]; then
  echo "ERROR: env file not found: $ENV_FILE" >&2
  echo "Run ./scripts/systemd/install_on_minipc.sh first." >&2
  exit 2
fi

if grep -q '^NAV_MODE=' "$ENV_FILE"; then
  sed -i "s/^NAV_MODE=.*/NAV_MODE=$MODE/" "$ENV_FILE"
else
  echo "NAV_MODE=$MODE" >> "$ENV_FILE"
fi

systemctl --user daemon-reload
systemctl --user restart gxu2026-nav-stack.service
echo "[switch_nav_mode] NAV_MODE=$MODE and restarted gxu2026-nav-stack.service"