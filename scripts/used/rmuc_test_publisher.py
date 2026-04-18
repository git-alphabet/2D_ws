#!/usr/bin/env python3
"""
RMUC 2026 裁判系统话题模拟器
=============================
模拟裁判系统和传感器发布的 10 个话题，用于测试 RMUC 行为树。
注意：robot_position 由 robot_position_bridge.py 发布（从 TF + Nav2 状态获取），
本模拟器不再发布该话题以避免冲突。

使用方式:
  1. 先在一个终端启动仿真:  ./scripts/sim_mapping.sh
  2. 再开另一个终端:
     docker exec -it gxu2026-nav-laptop bash
     source /ws/.buildcache/Alphabet/install/setup.bash
     python3 /ws/scripts/rmuc_test_publisher.py

     或直接:
     docker exec gxu2026-nav-laptop bash -c \
       "source /opt/ros/humble/setup.bash && \
        source /ws/.buildcache/Alphabet/install/setup.bash && \
        python3 /ws/scripts/used/rmuc_test_publisher.py"

参数:
  --ns       命名空间 (默认为空，裁判系统话题不带ns)
  --phase    比赛阶段 (0-5, 默认 4=比赛中)
  --remain   赛阶段剩余时间，秒 (默认 300)
  --hp       当前血量 (默认 400，设为 0 模拟战亡)
  --ammo     允许发弹量 (默认 200)
  --outpost-dead  前哨站是否被毁 (不加=存活)
  --detect-enemy  云台是否检测到敌人 (不加=未检测，用于基地威胁解除判定)
  --base-hp       基地当前血量 (默认 5000)
  --base-hp-drain 每秒基地血量下降速度 (默认 0)
  --enemy-near-base 雷达模拟敌人在基地附近 (配合 --base-hp-drain 触发基地威胁)
  --duration      运行时长秒数 (0=持续运行, 默认0)

复合指令示例（按序列模拟多阶段场景，用 --duration 代替 timeout）:
  # 满血2秒 → 自动切换到低血量
  python3 /ws/scripts/used/rmuc_test_publisher.py --hp=400 --duration=2 && python3 /ws/scripts/used/rmuc_test_publisher.py --hp=80

  # 满血3秒 → 低血量5秒 → 满血+检测到敌人
  python3 /ws/scripts/used/rmuc_test_publisher.py --hp=400 --duration=3 && python3 /ws/scripts/used/rmuc_test_publisher.py --hp=80 --duration=5 && python3 /ws/scripts/used/rmuc_test_publisher.py --hp=400 --detect-enemy

  # 满血2秒 → 前哨站被毁（触发目标切换到梯形高地）
  python3 /ws/scripts/used/rmuc_test_publisher.py --hp=400 --duration=2 && python3 /ws/scripts/used/rmuc_test_publisher.py --hp=400 --outpost-dead
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header

from sp_msgs.msg import (
    RMUCGameStatus,
    RMUCRobotStatus,
    RMUCRFIDStatus,
    RMUCSentryDecisionStatus,
    RMUCRobotBuff,
    RMUCProjectileAllowance,
    RMUCFieldStatus,
    RMUCEnemyMark,
    RMUCEnemyTracks,
    RMUCTeamHP,
    RMUCSentryCmd,
)


class RmucTestPublisher(Node):
    """发布所有 RMUC 裁判系统模拟话题。"""

    def __init__(self, args, namespace=""):
        super().__init__("rmuc_test_publisher", namespace=namespace)
        self.args = args

        # ── 发布者 (话题名不带 ns，由 node namespace 自动加前缀) ──
        self.pub_game = self.create_publisher(RMUCGameStatus, "game_status", 10)
        self.pub_robot = self.create_publisher(RMUCRobotStatus, "robot_status", 10)
        self.pub_rfid = self.create_publisher(RMUCRFIDStatus, "rfid_status", 10)
        # robot_position 由 robot_position_bridge.py 发布（从 TF + Nav2 状态获取），此处不再重复发布
        self.pub_sentry_decision = self.create_publisher(
            RMUCSentryDecisionStatus, "sentry_decision_status", 10
        )
        self.pub_robot_buff = self.create_publisher(RMUCRobotBuff, "robot_buff", 10)
        self.pub_projectile = self.create_publisher(
            RMUCProjectileAllowance, "projectile_allowance", 10
        )
        self.pub_field = self.create_publisher(RMUCFieldStatus, "field_status", 10)
        self.pub_enemy_mark = self.create_publisher(RMUCEnemyMark, "enemy_mark", 10)
        self.pub_team_hp = self.create_publisher(RMUCTeamHP, "team_hp", 10)

        # ── 雷达敌方跟踪 (用于模拟基地威胁进入条件) ──
        self.pub_radar_tracks = self.create_publisher(RMUCEnemyTracks, "radar/enemy_tracks", 10)

        # ── 订阅 BT 的 sentry_cmd，镜像 cmd_posture 到 sentry_decision_status ──
        self.current_posture = 3  # 默认移动姿态
        self.sub_sentry_cmd = self.create_subscription(
            RMUCSentryCmd, "sentry_cmd", self._sentry_cmd_cb, 10
        )

        # 倒计时状态
        self.remain_time = args.remain
        self.tick_count = 0

        # 基地血量状态 (用于模拟基地掉血)
        self.base_hp = float(args.base_hp)
        self.base_hp_drain = args.base_hp_drain  # 每秒下降量

        # 定时器: 主循环 10 Hz
        self.timer_10hz = self.create_timer(0.1, self.tick_10hz)
        # 1 Hz 话题
        self.timer_1hz = self.create_timer(1.0, self.tick_1hz)

        self.get_logger().info(
            f"RMUC 模拟器启动: phase={args.phase}, remain={args.remain}s, "
            f"hp={args.hp}, ammo={args.ammo}, "
            f"outpost_dead={args.outpost_dead}, detect_enemy={args.detect_enemy}, "
            f"base_hp={args.base_hp}, base_hp_drain={args.base_hp_drain}/s, "
            f"enemy_near_base={args.enemy_near_base}"
        )

    def _sentry_cmd_cb(self, msg: RMUCSentryCmd):
        if msg.cmd_posture in (1, 2, 3):
            self.current_posture = msg.cmd_posture

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
        self._pub_sentry_decision_status()
        self._pub_robot_buff()
        self._pub_projectile_allowance()
        self._pub_enemy_mark()
        self._pub_radar_tracks()

    def _pub_robot_status(self):
        msg = RMUCRobotStatus()
        msg.header = self._header()
        msg.current_hp = self.args.hp
        msg.max_hp = 600
        msg.shooter_heat = 30
        msg.ammo_allow = self.args.ammo
        msg.ammo_left = 300
        msg.base_hp_cur = int(self.base_hp)
        msg.base_hp_max = 5000
        msg.outpost_alive = not self.args.outpost_dead
        msg.is_detect_enemy = self.args.detect_enemy
        self.pub_robot.publish(msg)

    def _pub_sentry_decision_status(self):
        msg = RMUCSentryDecisionStatus()
        msg.header = self._header()
        msg.can_instant_respawn = True
        msg.instant_respawn_cost = 100
        msg.current_posture = self.current_posture  # 镜像 BT 的 sentry_cmd.cmd_posture
        msg.remote_ammo_count = 0
        msg.remote_heal_count = 0
        msg.exchanged_ammo_total = 0
        self.pub_sentry_decision.publish(msg)

    def _pub_robot_buff(self):
        msg = RMUCRobotBuff()
        msg.header = self._header()
        msg.heal_rate = 0
        msg.cool_value = 0
        msg.defense_pct = 0
        msg.vulnerability_pct = 0
        msg.attack_pct = 0
        self.pub_robot_buff.publish(msg)

    def _pub_projectile_allowance(self):
        msg = RMUCProjectileAllowance()
        msg.header = self._header()
        msg.fortress_ammo = 0
        msg.remaining_coins = 800
        self.pub_projectile.publish(msg)

    def _pub_enemy_mark(self):
        msg = RMUCEnemyMark()
        msg.header = self._header()
        msg.enemy_hero_vuln = False
        msg.enemy_engi_vuln = False
        msg.enemy_infantry3_vuln = False
        msg.enemy_infantry4_vuln = False
        msg.enemy_sentry_vuln = False
        self.pub_enemy_mark.publish(msg)

    def _pub_radar_tracks(self):
        msg = RMUCEnemyTracks()
        msg.header = self._header()
        if self.args.enemy_near_base:
            # 模拟一个敌人在 defend_anchor (2.76, -1.93) 附近 3m 处
            msg.enemy_count = 1
            msg.enemy_x = [2.76 + 2.0]  # ~2m 偏移，在 5m 阈值内
            msg.enemy_y = [-1.93 + 1.0]
        else:
            msg.enemy_count = 0
            msg.enemy_x = []
            msg.enemy_y = []
        self.pub_radar_tracks.publish(msg)

    # ────────── 1 Hz 话题 ──────────

    def tick_1hz(self):
        self._pub_game_status()
        self._pub_rfid_status()
        self._pub_field_status()
        self._pub_team_hp()

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

    def _pub_rfid_status(self):
        msg = RMUCRFIDStatus()
        msg.header = self._header()
        msg.rfid_supply = False
        msg.rfid_base_buff = False
        msg.rfid_outpost_buff = False
        msg.rfid_fortress_enemy = False
        msg.rfid_central_highland = False
        msg.rfid_ladder_highland = False
        self.pub_rfid.publish(msg)

    def _pub_field_status(self):
        msg = RMUCFieldStatus()
        msg.header = self._header()
        msg.small_energy_status = 0
        msg.big_energy_status = 0
        msg.central_highland = 0  # 未占领
        msg.ladder_highland = 0
        msg.fortress = 0
        msg.outpost_buff = 0
        msg.base_buff = False
        self.pub_field.publish(msg)

    def _pub_team_hp(self):
        msg = RMUCTeamHP()
        msg.header = self._header()
        msg.outpost_hp = 0 if self.args.outpost_dead else 1500
        msg.base_hp = int(self.base_hp)
        self.pub_team_hp.publish(msg)


def main():
    parser = argparse.ArgumentParser(description="RMUC 2026 裁判系统话题模拟器")
    parser.add_argument("--ns", default="/red_standard_robot1", help="命名空间 (仿真默认 /red_standard_robot1，实车用空字符串)")
    parser.add_argument("--phase", type=int, default=4, help="比赛阶段 (0-5, 4=比赛中)")
    parser.add_argument("--remain", type=int, default=420, help="阶段剩余时间 (秒)")
    parser.add_argument("--hp", type=int, default=400, help="当前血量")
    parser.add_argument("--ammo", type=int, default=300, help="允许发弹量")
    parser.add_argument("--outpost-dead", action="store_true", help="前哨站被毁")
    parser.add_argument("--detect-enemy", action="store_true", help="检测到敌人")
    parser.add_argument("--base-hp", type=int, default=5000, help="基地当前血量 (默认5000)")
    parser.add_argument("--base-hp-drain", type=float, default=0,
                        help="每秒基地血量下降速度 (默认0)")
    parser.add_argument("--enemy-near-base", action="store_true",
                        help="雷达模拟敌人在基地附近 (配合 --base-hp-drain 触发基地威胁)")
    parser.add_argument("--duration", type=float, default=0,
                        help="运行时长(秒), 0=持续运行直到Ctrl+C")
    args = parser.parse_args()

    rclpy.init()

    # 使用 namespace 让所有话题自动带前缀 (如 /red_standard_robot1/game_status)
    node = RmucTestPublisher(args, namespace=args.ns)

    try:
        node.get_logger().info(
            f"话题命名空间: {args.ns} (话题如 {args.ns}/game_status)"
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
