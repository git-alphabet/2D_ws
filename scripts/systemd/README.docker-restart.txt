Install:
  sudo bash scripts/systemd/install_docker_restart_service.sh

Config:
  sudo nano /etc/default/gxu2026-docker-restart

Check status:
  systemctl status gxu2026-docker-restart.service
  systemctl status gxu2026-docker-xauth.service
  cat /tmp/gxu2026-docker-gui/status
  cat /tmp/gxu2026-docker-gui/ready
  cat /tmp/gxu2026-docker-gui/session_fingerprint

Run once:
  sudo systemctl start gxu2026-docker-restart.service
