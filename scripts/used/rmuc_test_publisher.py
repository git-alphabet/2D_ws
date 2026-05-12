#!/usr/bin/env python3
"""
RMUC 2026 裁判系统话题模拟器
=============================
模拟裁判系统和传感器发布的话题，用于测试 RMUC 行为树。
注意：robot_position 由 robot_position_bridge.py 发布（从 TF + Nav2 状态获取），
本模拟器不再发布该话题以避免冲突。

活跃发布话题:
  /game_status       — RMUCGameStatus (1 Hz)
  /robot_status      — RMUCRobotStatus (10 Hz)
  /radar/enemy_tracks — RMUCEnemyTracks (10 Hz)
  /sentry_cmd        — RMUCSentryCmd (only 模式下手动发布)

使用方式:
  1. 先在一个终端启动仿真:  ./scripts/sim_mapping.sh
  2. 再开另一个终端:
     docker exec -it gxu2026-nav-laptop bash
     source /ws/.buildcache/Alphabet/install/setup.bash
     python3 /ws/scripts/rmuc_test_publisher.py --ns=/red_standard_robot1

      或直接:
     docker exec gxu2026-nav-laptop bash -c \
       "source /opt/ros/humble/setup.bash && \
        source /ws/.buildcache/Alphabet/install/setup.bash && \
          python3 /ws/scripts/rmuc_test_publisher.py"

参数:
    --ns       命名空间。默认 auto，会自动匹配当前导航/行为树所在命名空间
  --phase    比赛阶段 (0-5, 默认 4=比赛中)
  --remain   赛阶段剩余时间，秒 (默认 300)
  --hp       当前血量 (默认 400，设为 0 模拟战亡)
  --ammo     允许发弹量 (默认 300)
    --outpost-dead  我方前哨站是否被毁 (不加=存活)
    --enemy-outpost-destroyed  敌方前哨站是否被毁 (不加=未被毁)
  --detect-enemy  云台是否检测到敌人 (不加=未检测，用于基地威胁解除判定)
  --base-hp       基地当前血量 (默认 5000)
  --base-hp-drain 每秒基地血量下降速度 (默认 0)
  --enemy-near-base 雷达模拟敌人在基地附近 (配合 --base-hp-drain 触发基地威胁)
    --base-x        基地坐标 x (默认当前活动 RMUC 参数)
    --base-y        基地坐标 y (默认当前活动 RMUC 参数)
  --coins         剩余金币 (默认 800)
  --supply-delay  N秒后模拟补给区发放弹药 (0=不发放, 用于测试超时)
  --supply-amount 每次发放弹药量 (默认 100)
  --supply-repeat 是否每隔 supply-delay 秒重复发放 (不加=仅一次)
  --only          只发布指定话题。目前支持 sentry_cmd
  --posture       --only sentry_cmd 时发布的姿态 (1=进攻, 2=防御, 3=移动)
  --confirm-respawn
                  --only sentry_cmd 时发布确认复活
  --duration      运行时长秒数 (0=持续运行, 默认0)

复合指令示例（按序列模拟多阶段场景，用 --duration 代替 timeout）:
  # 满血2秒 → 自动切换到低血量
    python3 /ws/scripts/rmuc_test_publisher.py --hp=400 --duration=2 && python3 /ws/scripts/rmuc_test_publisher.py --hp=80

  # 满血3秒 → 低血量5秒 → 满血+检测到敌人
    python3 /ws/scripts/rmuc_test_publisher.py --hp=400 --duration=3 && python3 /ws/scripts/rmuc_test_publisher.py --hp=80 --duration=5 && python3 /ws/scripts/rmuc_test_publisher.py --hp=400 --detect-enemy

    # 敌方前哨站被毁且我方前哨站存活（触发目标切换到中央高地）
        python3 /ws/scripts/rmuc_test_publisher.py --hp=400 --duration=2 && python3 /ws/scripts/rmuc_test_publisher.py --hp=400 --enemy-outpost-destroyed

    # 敌方前哨站被毁且我方前哨站也被毁（触发目标切换到梯形高地）
        python3 /ws/scripts/rmuc_test_publisher.py --hp=400 --enemy-outpost-destroyed --outpost-dead

  # 低弹药+10秒后补给到账（测试补给成功路径）
    python3 /ws/scripts/rmuc_test_publisher.py --ammo=50 --supply-delay=10 --supply-amount=100

  # 低弹药+永不补给（测试30秒超时降级）
    python3 /ws/scripts/rmuc_test_publisher.py --ammo=50 --supply-delay=0

  # 低弹药+每60秒补给100发（模拟真实免费补给节奏）
    python3 /ws/scripts/rmuc_test_publisher.py --ammo=50 --supply-delay=60 --supply-amount=100 --supply-repeat

  # 只发布姿态指令到 /sentry_cmd，姿态 1=进攻 2=防御 3=移动
    python3 /ws/scripts/rmuc_test_publisher.py --ns=/red_standard_robot1 --only=sentry_cmd --posture=1

  # 只发布确认复活到 /sentry_cmd
    python3 /ws/scripts/rmuc_test_publisher.py --ns=/red_standard_robot1 --only=sentry_cmd --confirm-respawn
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool, Header

from sp_msgs.msg import (
    RMUCGameStatus,
    RMUCRobotStatus,
    RMUCEnemyTracks,
    RMUCSentryCmd,
)


class RmucTestPublisher(Node):
    """发布所有 RMUC 裁判系统模拟话题。"""

    def __init__(self, args, namespace=""):
        super().__init__("rmuc_test_publisher", namespace=namespace)
        self.args = args
        self.resolved_namespace = _normalize_namespace(namespace)

        # ── 发布者 (话题名不带 ns，由 node namespace 自动加前缀) ──
        self.pub_game = self.create_publisher(RMUCGameStatus, "game_status", 10)
        self.pub_robot = self.create_publisher(RMUCRobotStatus, "robot_status", 10)
        # robot_position 由 robot_position_bridge.py 发布（从 TF + Nav2 状态获取），此处不再重复发布
        # ── enable_power: rmua19_robot_base 必须收到此信号才会响应 cmd_vel ──
        # 路径镜像 Gazebo 裁判插件: /referee_system/{ns}/enable_power
        # 空 ns 时退化为 /referee_system/enable_power
        _ns = self.resolved_namespace.strip('/')
        _ep_topic = f"/referee_system/{_ns}/enable_power" if _ns else "/referee_system/enable_power"
        _latch_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_enable_power = self.create_publisher(Bool, _ep_topic, _latch_qos)
        # 立即发布一次，避免 robot_base 启动后等待第一次 timer
        _ep = Bool()
        _ep.data = True
        self.pub_enable_power.publish(_ep)

        # ── 雷达敌方跟踪 (用于模拟基地威胁进入条件) ──
        self.pub_radar_tracks = self.create_publisher(RMUCEnemyTracks, "radar/enemy_tracks", 10)

        # 倒计时状态
        self.remain_time = args.remain
        self.tick_count = 0

        # 基地血量状态 (用于模拟基地掉血)
        self.base_hp = float(args.base_hp)
        self.base_hp_drain = args.base_hp_drain  # 每秒下降量

        # 弹药补给模拟
        self.current_ammo = args.ammo  # 可动态变化
        self.supply_delivered = False  # 是否已发放（非repeat模式）
        self.supply_elapsed = 0.0  # 累计秒数

        # 定时器: 主循环 10 Hz
        self.timer_10hz = self.create_timer(0.1, self.tick_10hz)
        # 1 Hz 话题
        self.timer_1hz = self.create_timer(1.0, self.tick_1hz)

        self.get_logger().info(
            f"RMUC 模拟器启动: phase={args.phase}, remain={args.remain}s, "
            f"hp={args.hp}, ammo={args.ammo}, "
            f"outpost_dead={args.outpost_dead}, enemy_outpost_destroyed={args.enemy_outpost_destroyed}, "
            f"detect_enemy={args.detect_enemy}, "
            f"base_hp={args.base_hp}, base_hp_drain={args.base_hp_drain}/s, "
            f"enemy_near_base={args.enemy_near_base}, "
            f"coins={args.coins}, "
            f"supply_delay={args.supply_delay}s, supply_amount={args.supply_amount}, "
            f"supply_repeat={args.supply_repeat}"
        )

    def _header(self):
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        h.frame_id = "base_link"
        return h

    # ────────── 10 Hz 话题 ──────────

    def tick_10hz(self):
        self.tick_count += 1

        # 基地血量逐 tick 下降 (drain_per_tick = drain_per_sec / 10Hz)
        if self.base_hp_drain > 0 and self.base_hp > 0:
            self.base_hp = max(0.0, self.base_hp - self.base_hp_drain / 10.0)

        self._pub_robot_status()
        self._pub_radar_tracks()

    def _pub_robot_status(self):
        msg = RMUCRobotStatus()
        msg.header = self._header()
        msg.current_hp = self.args.hp
        msg.shooter_heat = 30
        msg.ammo_allow = self.current_ammo
        msg.outpost_hp = 0 if self.args.outpost_dead else 1500
        msg.base_hp = int(self.base_hp)
        # 1/2 代表前哨站仍存活；其余状态会被 BT 解释为 destroyed=true。
        msg.enemy_outpost_status = 4 if self.args.enemy_outpost_destroyed else 1
        msg.is_detect_enemy = self.args.detect_enemy
        self.pub_robot.publish(msg)

    def _pub_radar_tracks(self):
        msg = RMUCEnemyTracks()
        msg.header = self._header()
        if self.args.enemy_near_base:
            # 模拟一个敌人在当前活动基地坐标附近，确保命中基地威胁判定。
            msg.enemy_x = [self.args.base_x + 2.0]  # ~2m 偏移，在 5m 阈值内
            msg.enemy_y = [self.args.base_y + 1.0]
        else:
            msg.enemy_x = []
            msg.enemy_y = []
        self.pub_radar_tracks.publish(msg)

    # ────────── 1 Hz 话题 ──────────

    def tick_1hz(self):
        self._pub_game_status()
        # 持续发布 enable_power=True，确保 rmua19_robot_base 持续响应 cmd_vel
        _ep = Bool()
        _ep.data = True
        self.pub_enable_power.publish(_ep)

        # 弹药补给模拟
        if self.args.supply_delay > 0:
            self.supply_elapsed += 1.0
            if self.args.supply_repeat:
                # 每隔 supply_delay 秒发放一次
                if self.supply_elapsed >= self.args.supply_delay:
                    self.supply_elapsed = 0.0
                    self.current_ammo += self.args.supply_amount
                    self.get_logger().info(
                        f"🔄 补给发放: +{self.args.supply_amount} → ammo={self.current_ammo}"
                    )
            else:
                # 仅发放一次
                if not self.supply_delivered and self.supply_elapsed >= self.args.supply_delay:
                    self.supply_delivered = True
                    self.current_ammo += self.args.supply_amount
                    self.get_logger().info(
                        f"📦 补给到账: +{self.args.supply_amount} → ammo={self.current_ammo}"
                    )

        # 倒计时 (模拟比赛中)
        if self.args.phase == 4 and self.remain_time > 0:
            self.remain_time -= 1
            if self.remain_time % 30 == 0:
                self.get_logger().info(f"比赛剩余: {self.remain_time}s")

    def _pub_game_status(self):
        msg = RMUCGameStatus()
        msg.header = self._header()
        msg.game_progress = self.args.phase
        msg.stage_remain_time = self.remain_time
        self.pub_game.publish(msg)


class RmucSingleTopicPublisher(Node):
    """只发布一个 RMUC 测试话题，避免干扰其他链路。"""

    def __init__(self, args, namespace=""):
        super().__init__("rmuc_single_topic_publisher", namespace=namespace)
        self.args = args
        self.resolved_namespace = _normalize_namespace(namespace)
        self.tick_count = 0

        if args.only == "sentry_cmd":
            self.pub_sentry_cmd = self.create_publisher(RMUCSentryCmd, "sentry_cmd", 10)
            self.timer = self.create_timer(0.5, self._pub_sentry_cmd)
            self.get_logger().info(
                f"单话题模式: sentry_cmd, posture={args.posture} "
                f"confirm_respawn={args.confirm_respawn} "
                f"(topic={(self.resolved_namespace or '') + '/sentry_cmd' if self.resolved_namespace else '/sentry_cmd'})"
            )
        else:
            raise ValueError(f"unsupported --only value: {args.only}")

    def _header(self):
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        h.frame_id = "base_link"
        return h

    def _pub_sentry_cmd(self):
        self.tick_count += 1
        msg = RMUCSentryCmd()
        msg.header = self._header()
        msg.cmd_posture = int(self.args.posture)
        msg.cmd_confirm_respawn = bool(self.args.confirm_respawn)
        self.pub_sentry_cmd.publish(msg)
        if self.tick_count == 1 or self.tick_count % 10 == 0:
            self.get_logger().info(
                "发布 sentry_cmd: "
                f"cmd_posture={msg.cmd_posture}, "
                f"cmd_confirm_respawn={msg.cmd_confirm_respawn}"
            )


def _normalize_namespace(namespace: str) -> str:
    namespace = (namespace or "").strip()
    if not namespace or namespace == "/":
        return ""
    if not namespace.startswith("/"):
        namespace = f"/{namespace}"
    return namespace.rstrip("/")


def _resolve_namespace(namespace_arg: str) -> str:
    normalized_arg = _normalize_namespace(namespace_arg)
    if namespace_arg.strip().lower() != "auto":
        return normalized_arg

    probe = Node("rmuc_test_publisher_probe")
    try:
        candidate_names = {
            "rm_behavior_tree",
            "bt_navigator",
            "controller_server",
            "planner_server",
            "behavior_server",
        }
        candidate_namespaces = []
        for node_name, node_namespace in probe.get_node_names_and_namespaces():
            if node_name in candidate_names:
                normalized_ns = _normalize_namespace(node_namespace)
                candidate_namespaces.append(normalized_ns)

        unique_candidates = sorted(set(candidate_namespaces))
        non_root_candidates = [item for item in unique_candidates if item]

        if len(non_root_candidates) == 1:
            return non_root_candidates[0]
        if len(unique_candidates) == 1:
            return unique_candidates[0]
        if not unique_candidates:
            return ""

        preferred_candidates = [item for item in non_root_candidates if "red_standard_robot1" in item]
        if len(preferred_candidates) == 1:
            return preferred_candidates[0]

        probe.get_logger().warning(
            "检测到多个候选命名空间 %s，rmuc_test_publisher 将回退到根命名空间；"
            "如需指定，请显式传入 --ns",
            unique_candidates,
        )
        return ""
    finally:
        probe.destroy_node()


def main():
    parser = argparse.ArgumentParser(description="RMUC 2026 裁判系统话题模拟器")
    parser.add_argument(
        "--ns",
        default="auto",
        help="命名空间；默认 auto，自动匹配当前导航/行为树命名空间",
    )
    parser.add_argument("--phase", type=int, default=4, help="比赛阶段 (0-5, 4=比赛中)")
    parser.add_argument("--remain", type=int, default=420, help="阶段剩余时间 (秒)")
    parser.add_argument("--hp", type=int, default=400, help="当前血量")
    parser.add_argument("--ammo", type=int, default=300, help="允许发弹量")
    parser.add_argument("--outpost-dead", action="store_true", help="我方前哨站被毁")
    parser.add_argument("--enemy-outpost-destroyed", action="store_true",
                        help="敌方前哨站被毁")
    parser.add_argument("--detect-enemy", action="store_true", help="检测到敌人")
    parser.add_argument("--base-hp", type=int, default=5000, help="基地当前血量 (默认5000)")
    parser.add_argument("--base-hp-drain", type=float, default=0,
                        help="每秒基地血量下降速度 (默认0)")
    parser.add_argument("--enemy-near-base", action="store_true",
                        help="雷达模拟敌人在基地附近 (配合 --base-hp-drain 触发基地威胁)")
    parser.add_argument("--base-x", type=float, default=-2.3532,
                        help="基地坐标 x (默认当前活动 RMUC 参数)")
    parser.add_argument("--base-y", type=float, default=-2.0007,
                        help="基地坐标 y (默认当前活动 RMUC 参数)")
    parser.add_argument("--coins", type=int, default=800,
                        help="剩余金币 (默认800)")
    parser.add_argument("--supply-delay", type=float, default=0,
                        help="N秒后模拟补给区发放弹药 (0=不发放，用于测试30s超时)")
    parser.add_argument("--supply-amount", type=int, default=100,
                        help="每次发放弹药量 (默认100)")
    parser.add_argument("--supply-repeat", action="store_true",
                        help="每隔 supply-delay 秒重复发放 (不加=仅发放一次)")
    parser.add_argument("--only", choices=["sentry_cmd"], default="",
                        help="只发布指定话题；目前支持 sentry_cmd")
    parser.add_argument("--posture", type=int, choices=[1, 2, 3], default=3,
                        help="--only sentry_cmd 时发布的姿态 (1=进攻, 2=防御, 3=移动)")
    parser.add_argument("--confirm-respawn", action="store_true",
                        help="--only sentry_cmd 时将 cmd_confirm_respawn 置为 true")
    parser.add_argument("--duration", type=float, default=0,
                        help="运行时长(秒), 0=持续运行直到Ctrl+C")
    args = parser.parse_args()

    rclpy.init()

    resolved_namespace = _resolve_namespace(args.ns)

    # 使用 namespace 让所有话题自动带前缀 (如 /red_standard_robot1/game_status)
    if args.only:
        node = RmucSingleTopicPublisher(args, namespace=resolved_namespace)
    else:
        node = RmucTestPublisher(args, namespace=resolved_namespace)

    try:
        node.get_logger().info(
            f"话题命名空间: {resolved_namespace or '/'} "
            f"(请求值: {args.ns})"
        )
        if args.duration > 0:
            node.get_logger().info(f"将在 {args.duration}s 后自动退出")
            node.create_timer(args.duration, lambda: rclpy.shutdown())
        else:
            node.get_logger().info("Ctrl+C 停止模拟")
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
