#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ========== 实车调试模式（放脚本顶端改这里）==========
# 可选: mapping | navigation
MODE="mapping"

# PlotJuggler 开关与命令
ENABLE_PLOTJUGGLER="${ENABLE_PLOTJUGGLER:-1}"
PLOTJUGGLER_CMD="${PLOTJUGGLER_CMD:-plotjuggler}"
PLOTJUGGLER_LAYOUT="${PLOTJUGGLER_LAYOUT:-}"
PLOTJUGGLER_LOG="${PLOTJUGGLER_LOG:-/tmp/reality_debug_plotjuggler.log}"
PLOTJUGGLER_WAIT_SEC="${PLOTJUGGLER_WAIT_SEC:-2}"

# PlotJuggler 启动前显式 source，避免外层终端未 source 导致找不到命令
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
OVERLAY_SETUP="${OVERLAY_SETUP:-$SCRIPT_DIR/../install/setup.bash}"
# ================================================

# 与现有实车脚本保持一致：默认单终端
export NO_NEW_TERMINAL="${NO_NEW_TERMINAL:-1}"
export KILL_EXISTING="${KILL_EXISTING:-1}"

plot_pid=""
main_pid=""

cleanup() {
    if [[ -n "$plot_pid" ]] && kill -0 "$plot_pid" 2>/dev/null; then
        kill "$plot_pid" 2>/dev/null || true
    fi
}

forward_signal() {
    local sig="$1"
    if [[ -n "$main_pid" ]] && kill -0 "$main_pid" 2>/dev/null; then
        kill "-$sig" "$main_pid" 2>/dev/null || kill "$main_pid" 2>/dev/null || true
    fi
}

trap 'forward_signal INT' INT
trap 'forward_signal TERM' TERM
trap cleanup EXIT

start_plotjuggler() {
    : > "$PLOTJUGGLER_LOG"

    local launch_script
    launch_script=$(cat <<'EOF'
set -e
if [[ -f "$ROS_SETUP" ]]; then
    source "$ROS_SETUP"
fi
if [[ -f "$OVERLAY_SETUP" ]]; then
    source "$OVERLAY_SETUP"
fi

if command -v "$PLOTJUGGLER_CMD" >/dev/null 2>&1; then
    if [[ -n "$PLOTJUGGLER_LAYOUT" ]]; then
        exec "$PLOTJUGGLER_CMD" -l "$PLOTJUGGLER_LAYOUT"
    fi
    exec "$PLOTJUGGLER_CMD"
fi

if command -v ros2 >/dev/null 2>&1; then
    if [[ -n "$PLOTJUGGLER_LAYOUT" ]]; then
        exec ros2 run plotjuggler plotjuggler -l "$PLOTJUGGLER_LAYOUT"
    fi
    exec ros2 run plotjuggler plotjuggler
fi

echo "[reality_debug] neither '$PLOTJUGGLER_CMD' nor 'ros2 run plotjuggler plotjuggler' is available"
exit 127
EOF
)

    ROS_SETUP="$ROS_SETUP" \
    OVERLAY_SETUP="$OVERLAY_SETUP" \
    PLOTJUGGLER_CMD="$PLOTJUGGLER_CMD" \
    PLOTJUGGLER_LAYOUT="$PLOTJUGGLER_LAYOUT" \
    bash -lc "$launch_script" >>"$PLOTJUGGLER_LOG" 2>&1 &
    plot_pid=$!

    sleep "$PLOTJUGGLER_WAIT_SEC"
    if kill -0 "$plot_pid" 2>/dev/null; then
        echo "[reality_debug] plotjuggler started, pid=$plot_pid, log=$PLOTJUGGLER_LOG"
    else
        echo "[reality_debug] ERROR: plotjuggler failed, check log: $PLOTJUGGLER_LOG" >&2
        tail -n 40 "$PLOTJUGGLER_LOG" >&2 || true
    fi
}

if [[ "$ENABLE_PLOTJUGGLER" == "1" ]]; then
    start_plotjuggler
fi

case "$MODE" in
    mapping)
        target_script="$SCRIPT_DIR/mapping.sh"
        ;;
    navigation)
        target_script="$SCRIPT_DIR/start_navigation.sh"
        ;;
    *)
        echo "[reality_debug] ERROR: MODE must be 'mapping' or 'navigation', got '$MODE'" >&2
        exit 1
        ;;
esac

"$target_script" "$@" &
main_pid=$!
wait "$main_pid"
