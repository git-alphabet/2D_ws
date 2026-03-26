#!/usr/bin/env bash
set -euo pipefail

MODE="navigation"
ENABLE_NAV=1
ENABLE_AUTO_AIM=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --mode)
      MODE="${2:-}"
      shift 2
      ;;
    --enable-nav)
      ENABLE_NAV="${2:-}"
      shift 2
      ;;
    --enable-auto-aim)
      ENABLE_AUTO_AIM="${2:-}"
      shift 2
      ;;
    -h|--help)
      cat <<'EOF'
Usage: ./scripts/systemd/install_on_minipc.sh [options]

Options:
  --mode <navigation|mapping>    Set nav startup mode (default: navigation)
  --enable-nav <0|1>             Enable nav systemd service (default: 1)
  --enable-auto-aim <0|1>        Enable auto-aim systemd service (default: 0)
EOF
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      exit 2
      ;;
  esac
done

if [[ "$MODE" != "navigation" && "$MODE" != "mapping" ]]; then
  echo "ERROR: --mode must be navigation or mapping" >&2
  exit 2
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

if [[ ! -f "$WS_DIR/docker/compose.dev.yml" ]]; then
  echo "ERROR: workspace seems invalid: $WS_DIR" >&2
  exit 2
fi

mkdir -p "$HOME/.config/systemd/user"
mkdir -p "$HOME/.config/gxu2026"

install -m 0644 "$SCRIPT_DIR/units/gxu2026-nav-stack.service" "$HOME/.config/systemd/user/gxu2026-nav-stack.service"
install -m 0644 "$SCRIPT_DIR/units/gxu2026-auto-aim.service" "$HOME/.config/systemd/user/gxu2026-auto-aim.service"

cat > "$HOME/.config/gxu2026/nav-stack.env" <<EOF
NAV_MODE=$MODE
START_RVIZ=1
KILL_EXISTING=1
NO_NEW_TERMINAL=1
EOF

chmod +x "$SCRIPT_DIR/nav_mode_launcher.sh"
chmod +x "$SCRIPT_DIR/service_ctl.sh"
chmod +x "$SCRIPT_DIR/switch_nav_mode.sh"
chmod +x "$SCRIPT_DIR/use_navigation.sh"
chmod +x "$SCRIPT_DIR/use_mapping.sh"
chmod +x "$SCRIPT_DIR/nav_start.sh"
chmod +x "$SCRIPT_DIR/nav_stop.sh"
chmod +x "$SCRIPT_DIR/mapping_stop.sh"
chmod +x "$SCRIPT_DIR/autoaim_start.sh"
chmod +x "$SCRIPT_DIR/autoaim_stop.sh"
chmod +x "$SCRIPT_DIR/all_stop.sh"
chmod +x "$SCRIPT_DIR/status.sh"

systemctl --user daemon-reload

if [[ "$ENABLE_NAV" == "1" ]]; then
  systemctl --user enable gxu2026-nav-stack.service
  systemctl --user restart gxu2026-nav-stack.service
else
  systemctl --user disable gxu2026-nav-stack.service || true
  systemctl --user stop gxu2026-nav-stack.service || true
fi

if [[ "$ENABLE_AUTO_AIM" == "1" ]]; then
  systemctl --user enable gxu2026-auto-aim.service
  systemctl --user restart gxu2026-auto-aim.service
else
  systemctl --user disable gxu2026-auto-aim.service || true
  systemctl --user stop gxu2026-auto-aim.service || true
fi

echo "[install_on_minipc] Done."
echo "[install_on_minipc] Nav mode: $MODE"
echo "[install_on_minipc] Auto aim enabled: $ENABLE_AUTO_AIM"
echo "[install_on_minipc] Check status:" 
echo "  systemctl --user status gxu2026-nav-stack.service"
echo "  systemctl --user status gxu2026-auto-aim.service"