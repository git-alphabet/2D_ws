#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 || $# -gt 2 ]]; then
  cat <<'EOF' >&2
Usage: ./scripts/systemd/service_ctl.sh <start|stop|restart|enable|disable|status> [nav|autoaim|all]

Default target: nav
EOF
  exit 2
fi

ACTION="$1"
TARGET="${2:-nav}"

case "$TARGET" in
  nav)
    SERVICES=(gxu2026-nav-stack.service)
    ;;
  autoaim)
    SERVICES=(gxu2026-auto-aim.service)
    ;;
  all)
    SERVICES=(gxu2026-nav-stack.service gxu2026-auto-aim.service)
    ;;
  *)
    echo "ERROR: invalid target '$TARGET', expected nav|autoaim|all" >&2
    exit 2
    ;;
esac

case "$ACTION" in
  start|stop|restart|enable|disable|status)
    ;;
  *)
    echo "ERROR: invalid action '$ACTION'" >&2
    exit 2
    ;;
esac

if [[ "$ACTION" == "enable" || "$ACTION" == "disable" ]]; then
  for svc in "${SERVICES[@]}"; do
    systemctl --user "$ACTION" "$svc" || true
  done
  exit 0
fi

if [[ "$ACTION" == "status" ]]; then
  for svc in "${SERVICES[@]}"; do
    echo "===== $svc ====="
    systemctl --user status "$svc" --no-pager || true
  done
  exit 0
fi

for svc in "${SERVICES[@]}"; do
  systemctl --user "$ACTION" "$svc"
done