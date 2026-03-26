#!/bin/bash
# =============================================================================
# pub_topic.sh — 仿真裁判系统话题发布器
# 默认命名空间 red_standard_robot1
# 自定义命名空间 NS=blue_standard_robot1 bash scripts/pub_topic.sh
# 自定义血量 HP=200 bash scripts/pub_topic.sh
# 用途：在 Gazebo 仿真中模拟裁判系统，发布行为树所需的所有话题
# =============================================================================
set -eo pipefail

# ── 默认参数（可通过环境变量覆盖） ──────────────────────────────
NS="${NS:-red_standard_robot1}"          # 命名空间（与 launch 默认值一致）
HP="${HP:-400}"                          # 当前血量
GAME_PROGRESS="${GAME_PROGRESS:-4}"      # 比赛阶段 (4=比赛中)
REMAIN_TIME="${REMAIN_TIME:-180}"        # 剩余时间 (秒)
DETECT_ENEMY="${DETECT_ENEMY:-false}"    # 是否检测到敌人
SHOOTER_HEAT="${SHOOTER_HEAT:-0}"        # 枪口热量

# ── Source 工作空间 ──────────────────────────────────────────────
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SETUP="$WS_DIR/install/setup.bash"
if [[ -f "$SETUP" ]]; then
    source "$SETUP"
else
    echo "[pub_topic] WARNING: $SETUP not found, assuming already sourced"
fi

echo "============================================"
echo "  RMUL 仿真裁判系统"
echo "============================================"
echo "  命名空间:   /$NS"
echo "  血量:       $HP"
echo "  比赛阶段:   $GAME_PROGRESS"
echo "  剩余时间:   ${REMAIN_TIME}s"
echo "  检测敌人:   $DETECT_ENEMY"
echo "  枪口热量:   $SHOOTER_HEAT"
echo "============================================"
echo ""
echo "  话题列表:"
echo "    /$NS/game_status     (1 Hz)"
echo "    /$NS/robot_status    (10 Hz)"
echo ""
echo "  Ctrl+C 停止所有发布"
echo "============================================"

# ── 清理函数 ─────────────────────────────────────────────────────
PIDS=()
cleanup() {
    echo ""
    echo "[sim_referee] Stopping..."
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null || true
    done
    wait 2>/dev/null
    echo "[sim_referee] Done."
}
trap cleanup EXIT INT TERM

# ── 发布 game_status (1 Hz) ─────────────────────────────────────
ros2 topic pub /"$NS"/game_status sp_msgs/msg/RMUL \
    "{game_progress: $GAME_PROGRESS, stage_remain_time: $REMAIN_TIME}" \
    --rate 1 --qos-reliability reliable &
PIDS+=($!)

# ── 发布 robot_status (10 Hz) ───────────────────────────────────
ros2 topic pub /"$NS"/robot_status sp_msgs/msg/RMUL \
    "{current_hp: $HP, shooter_heat: $SHOOTER_HEAT, is_detect_enemy: $DETECT_ENEMY, is_attacked: 0}" \
    --rate 10 --qos-reliability reliable &
PIDS+=($!)

echo "[pub_topic] All topics publishing. Waiting..."
wait
