#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

if [[ "${EUID}" -ne 0 ]]; then
  echo "Please run as root: sudo bash scripts/systemd/install_docker_xauth_service.sh" >&2
  exit 1
fi

install -d -m 0755 /tmp/gxu2026-docker-gui
install -m 0755 "$SCRIPT_DIR/docker_xauth_sync.sh" /usr/local/sbin/gxu2026-docker-xauth-sync
install -m 0644 "$SCRIPT_DIR/gxu2026-docker-xauth.service" /etc/systemd/system/gxu2026-docker-xauth.service
install -m 0644 "$SCRIPT_DIR/gxu2026-docker-xauth.timer" /etc/systemd/system/gxu2026-docker-xauth.timer
install -m 0644 "$SCRIPT_DIR/gxu2026-docker-xauth.path" /etc/systemd/system/gxu2026-docker-xauth.path

systemctl daemon-reload
systemctl enable --now gxu2026-docker-xauth.timer
systemctl enable --now gxu2026-docker-xauth.path
systemctl start gxu2026-docker-xauth.service

echo "Installed and started: gxu2026-docker-xauth.service + timer + path"
echo "Check path with: systemctl status gxu2026-docker-xauth.path"
echo "Check timer with: systemctl status gxu2026-docker-xauth.timer"
