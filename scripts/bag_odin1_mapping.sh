#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ── odin1 bag 回放建图配置 ────────────────────────────────────────
export REALITY_USE_SIM_TIME="True"
export AUTO_RECORD_BAG="0"

# odin1 建图：use_robot_state_pub 补全 tf_static，use_sim_time 和 bag 时钟同步
# 关闭 odin_driver、mid360_driver，bag 提供传感器数据
# 关闭 publish_static_map_tf，避免和 SLAM 发布的 map->odom 冲突
export MAPPING_CMD="ros2 launch gxu2026_nav_bringup rm_navigation_reality_launch.py \
    slam:=True \
    use_robot_state_pub:=True \
    use_sim_time:=True \
    use_odin_driver:=False \
    use_mid360_driver:=False \
    publish_static_map_tf:=False"

# ── 参数解析 ──────────────────────────────────────────────────────
# 用法: odin1_mapping_bag.sh <bag目录> [odin1_mapping.sh 参数...]
#       odin1_mapping_bag.sh              # 不回放 bag，只启动建图程序
BAG_DIR="${1:-}"
if [ -n "$BAG_DIR" ]; then
    shift
fi

BAG_RATE="${BAG_RATE:-1.0}"
BAG_LOOP="${BAG_LOOP:-1}"

_bag_pid=""

_cleanup() {
    if [ -n "$_bag_pid" ] && kill -0 "$_bag_pid" 2>/dev/null; then
        echo "[odin1_mapping_bag] Stopping bag replay (PID $_bag_pid)..."
        kill "$_bag_pid" 2>/dev/null || true
        wait "$_bag_pid" 2>/dev/null || true
    fi
}
trap _cleanup EXIT INT TERM

# ── 启动建图程序（先启动，等待 /clock）──────────────────────────
"$SCRIPT_DIR/odin1_mapping.sh" "$@" &
_mapping_pid=$!

# ── 启动 bag 回放 ──────────────────────────────────────────────
if [ -n "$BAG_DIR" ]; then
    if [ ! -d "$BAG_DIR" ]; then
        candidate="$SCRIPT_DIR/../bags/$BAG_DIR"
        if [ -d "$candidate" ]; then
            BAG_DIR="$candidate"
        fi
    fi

    if [ ! -d "$BAG_DIR" ]; then
        echo "[odin1_mapping_bag] ERROR: bag directory not found: $BAG_DIR" >&2
        kill $_mapping_pid 2>/dev/null || true
        exit 1
    fi

    BAG_CMD="python3 $SCRIPT_DIR/../src/ros2_bag_tools/scripts/play_bag $BAG_DIR --raw --rate $BAG_RATE"
    if [ "$BAG_LOOP" = "1" ]; then
        BAG_CMD="$BAG_CMD --loop"
    fi

    echo "[odin1_mapping_bag] Starting bag replay (raw mode, rate=$BAG_RATE, loop=$BAG_LOOP): $BAG_DIR"
    _branch="$(git -C "$SCRIPT_DIR" rev-parse --abbrev-ref HEAD 2>/dev/null || echo default)"
    _branch_safe="$(echo "$_branch" | sed 's#[^A-Za-z0-9._-]#_#g')"
    _install_setup="$SCRIPT_DIR/../.buildcache/$_branch_safe/install/setup.bash"
    [ -f "$_install_setup" ] || _install_setup="$SCRIPT_DIR/../install/setup.bash"
    bash -c "source /opt/ros/humble/setup.bash && source $_install_setup 2>/dev/null; $BAG_CMD" &
    _bag_pid=$!
    echo "[odin1_mapping_bag] Bag replay PID: $_bag_pid"
fi

wait $_mapping_pid 2>/dev/null || true
