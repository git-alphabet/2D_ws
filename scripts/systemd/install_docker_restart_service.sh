#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

if [[ "${EUID}" -ne 0 ]]; then
  echo "Please run as root: sudo bash scripts/systemd/install_docker_restart_service.sh" >&2
  exit 1
fi

install -m 0755 "$SCRIPT_DIR/gxu2026-docker-restart.sh" /usr/local/sbin/gxu2026-docker-restart
install -m 0644 "$SCRIPT_DIR/gxu2026-docker-restart.service" /etc/systemd/system/gxu2026-docker-restart.service

if [[ ! -f /etc/default/gxu2026-docker-restart ]]; then
  install -m 0644 "$SCRIPT_DIR/gxu2026-docker-restart.env" /etc/default/gxu2026-docker-restart
fi

systemctl daemon-reload
systemctl disable --now gxu2026-docker-restart.timer >/dev/null 2>&1 || true
systemctl start gxu2026-docker-restart.service

echo "Installed and started: gxu2026-docker-restart.service"
echo "Trigger source: gxu2026-docker-xauth.service (after GUI is ready)"
echo "Edit containers in: /etc/default/gxu2026-docker-restart"
