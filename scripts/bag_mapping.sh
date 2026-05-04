#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ── bag 回放建图配置 ──────────────────────────────────────────────
# 仿真时钟 + 关闭自动录包
export REALITY_USE_SIM_TIME="True"
export AUTO_RECORD_BAG="0"

# 关闭 mid360_driver（bag 提供传感器数据）
# 开启 robot_state_publisher（补全 bag 可能缺失的 /tf_static）
# 开启 static_tf_map2odom（SLAM Toolbox transform_publish_period=0 不发布 map->odom，需要 static 补上）
# use_sim_time=True：所有节点用 /clock 时间，和 bag 时间戳一致
# bag 回放用 --raw 模式过滤 /tf 和 /tf_static，不会与 robot_state_publisher 冲突
export MAPPING_CMD="ros2 launch gxu2026_nav_bringup rm_navigation_reality_launch.py \
    slam:=True \
    use_robot_state_pub:=True \
    use_mid360_driver:=False \
    use_static_tf_map2odom:=True \
    use_sim_time:=True"

# ── 参数解析 ──────────────────────────────────────────────────────
# 用法: bag_mapping.sh <bag目录> [mapping.sh 参数...]
#       bag_mapping.sh                # 不回放 bag，只启动建图程序
BAG_DIR="${1:-}"
if [ -n "$BAG_DIR" ]; then
    shift  # 消耗第一个参数，剩余的传给 mapping.sh
fi

BAG_RATE="${BAG_RATE:-1.0}"
BAG_LOOP="${BAG_LOOP:-1}"

_bag_pid=""

_cleanup() {
    if [ -n "$_bag_pid" ] && kill -0 "$_bag_pid" 2>/dev/null; then
        echo "[bag_mapping] Stopping bag replay (PID $_bag_pid)..."
        kill "$_bag_pid" 2>/dev/null || true
        wait "$_bag_pid" 2>/dev/null || true
    fi
}
trap _cleanup EXIT INT TERM

# ── 启动建图程序（先启动程序，等就绪后再启动 bag）──────────────
# use_sim_time=True 时，程序会等待 /clock，bag 启动后才开始运行
"$SCRIPT_DIR/mapping.sh" "$@" &
_mapping_pid=$!

# ── 启动 bag 回放 ──────────────────────────────────────────────
if [ -n "$BAG_DIR" ]; then
    # 兼容相对路径
    if [ ! -d "$BAG_DIR" ]; then
        candidate="$SCRIPT_DIR/../bags/$BAG_DIR"
        if [ -d "$candidate" ]; then
            BAG_DIR="$candidate"
        fi
    fi

    if [ ! -d "$BAG_DIR" ]; then
        echo "[bag_mapping] ERROR: bag directory not found: $BAG_DIR" >&2
        kill $_mapping_pid 2>/dev/null || true
        exit 1
    fi

    # --raw: 只回放 bag_topics_raw.txt 中的话题（不含 /tf /tf_static）
    # --clock: 发布仿真时钟
    BAG_CMD="python3 $SCRIPT_DIR/../src/ros2_bag_tools/scripts/play_bag $BAG_DIR --raw --rate $BAG_RATE"
    if [ "$BAG_LOOP" = "1" ]; then
        BAG_CMD="$BAG_CMD --loop"
    fi

    echo "[bag_mapping] Starting bag replay (raw mode, rate=$BAG_RATE, loop=$BAG_LOOP): $BAG_DIR"
    # 获取当前分支对应的 install 路径
    _branch="$(git -C "$SCRIPT_DIR" rev-parse --abbrev-ref HEAD 2>/dev/null || echo default)"
    _branch_safe="$(echo "$_branch" | sed 's#[^A-Za-z0-9._-]#_#g')"
    _install_setup="$SCRIPT_DIR/../.buildcache/$_branch_safe/install/setup.bash"
    [ -f "$_install_setup" ] || _install_setup="$SCRIPT_DIR/../install/setup.bash"
    bash -c "source /opt/ros/humble/setup.bash && source $_install_setup 2>/dev/null; $BAG_CMD" &
    _bag_pid=$!
    echo "[bag_mapping] Bag replay PID: $_bag_pid"
fi

# 等待建图程序结束
wait $_mapping_pid 2>/dev/null || true
