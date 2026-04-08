#!/usr/bin/env python3
"""
RMUC 2026 裁判系统话题模拟器
=============================
模拟裁判系统和传感器发布的全部 11 个话题，用于测试 RMUC 行为树。

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
        python3 /ws/scripts/rmuc_test_publisher.py"

参数:
  --ns       命名空间 (默认 /red_standard_robot1)
  --phase    比赛阶段 (0-5, 默认 4=比赛中)
  --remain   赛阶段剩余时间，秒 (默认 300)
  --hp       当前血量 (默认 400)
  --ammo     允许发弹量 (默认 200)
  --dead     是否战亡 (默认 false)
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
    RMUCRobotPosition,
    RMUCSentryDecisionStatus,
    RMUCRobotBuff,
    RMUCProjectileAllowance,
    RMUCFieldStatus,
    RMUCEnemyMark,
    RMUCTeamPositions,
    RMUCTeamHP,
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
        self.pub_position = self.create_publisher(RMUCRobotPosition, "robot_position", 10)
        self.pub_sentry_decision = self.create_publisher(
            RMUCSentryDecisionStatus, "sentry_decision_status", 10
        )
        self.pub_robot_buff = self.create_publisher(RMUCRobotBuff, "robot_buff", 10)
        self.pub_projectile = self.create_publisher(
            RMUCProjectileAllowance, "projectile_allowance", 10
        )
        self.pub_field = self.create_publisher(RMUCFieldStatus, "field_status", 10)
        self.pub_enemy_mark = self.create_publisher(RMUCEnemyMark, "enemy_mark", 10)
        self.pub_team_pos = self.create_publisher(RMUCTeamPositions, "team_positions", 10)
        self.pub_team_hp = self.create_publisher(RMUCTeamHP, "team_hp", 10)

        # 倒计时状态
        self.remain_time = args.remain
        self.tick_count = 0

        # 定时器: 主循环 10 Hz
        self.timer_10hz = self.create_timer(0.1, self.tick_10hz)
        # 1 Hz 话题
        self.timer_1hz = self.create_timer(1.0, self.tick_1hz)
        # 50 Hz 位置
        self.timer_50hz = self.create_timer(0.02, self.tick_50hz)

        self.get_logger().info(
            f"RMUC 模拟器启动: phase={args.phase}, remain={args.remain}s, "
            f"hp={args.hp}, ammo={args.ammo}, dead={args.dead}"
        )

    def _header(self):
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        h.frame_id = "base_link"
        return h

    # ────────── 10 Hz 话题 ──────────

    def tick_10hz(self):
        self.tick_count += 1
        self._pub_robot_status()
        self._pub_sentry_decision_status()
        self._pub_robot_buff()
        self._pub_projectile_allowance()
        self._pub_enemy_mark()

    def _pub_robot_status(self):
        msg = RMUCRobotStatus()
        msg.header = self._header()
        msg.current_hp = self.args.hp
        msg.max_hp = 600
        msg.shooter_heat = 30
        msg.heat_limit = 240
        msg.cooling_rate = 40
        msg.ammo_allow = self.args.ammo
        msg.ammo_left = 300
        msg.shooter_power_output = not self.args.dead
        msg.is_dead = self.args.dead
        msg.can_remote_heal = True
        msg.can_remote_ammo = True
        msg.team_coins = 800
        msg.can_respawn = False
        msg.respawn_countdown_s = 0
        msg.base_hp_cur = 5000
        msg.base_hp_max = 5000
        msg.outpost_alive = True
        msg.is_detect_enemy = False
        self.pub_robot.publish(msg)

    def _pub_sentry_decision_status(self):
        msg = RMUCSentryDecisionStatus()
        msg.header = self._header()
        msg.can_free_respawn = False
        msg.can_instant_respawn = True
        msg.instant_respawn_cost = 100
        msg.current_posture = 3  # 移动姿态
        msg.remote_ammo_count = 0
        msg.remote_heal_count = 0
        msg.exchanged_ammo_total = 0
        msg.can_activate_energy = False
        self.pub_sentry_decision.publish(msg)

    def _pub_robot_buff(self):
        msg = RMUCRobotBuff()
        msg.header = self._header()
        msg.heal_rate = 0
        msg.cool_value = 0
        msg.defense_pct = 0
        msg.vulnerability_pct = 0
        msg.attack_pct = 0
        msg.remaining_energy = 0
        self.pub_robot_buff.publish(msg)

    def _pub_projectile_allowance(self):
        msg = RMUCProjectileAllowance()
        msg.header = self._header()
        msg.ammo_17mm = self.args.ammo
        msg.ammo_42mm = 0
        msg.remaining_coins = 800
        msg.fortress_ammo = 0
        self.pub_projectile.publish(msg)

    def _pub_enemy_mark(self):
        msg = RMUCEnemyMark()
        msg.header = self._header()
        msg.enemy_hero_vuln = False
        msg.enemy_engi_vuln = False
        msg.enemy_infantry3_vuln = False
        msg.enemy_infantry4_vuln = False
        msg.enemy_sentry_vuln = False
        msg.ally_hero_marked = False
        msg.ally_engi_marked = False
        msg.ally_infantry3_marked = False
        msg.ally_infantry4_marked = False
        msg.ally_sentry_marked = False
        self.pub_enemy_mark.publish(msg)

    # ────────── 1 Hz 话题 ──────────

    def tick_1hz(self):
        self._pub_game_status()
        self._pub_rfid_status()
        self._pub_field_status()
        self._pub_team_positions()
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
        msg.rfid_fortress_ally = False
        msg.rfid_fortress_enemy = False
        msg.rfid_central_highland = False
        msg.rfid_ladder_highland = False
        self.pub_rfid.publish(msg)

    def _pub_field_status(self):
        msg = RMUCFieldStatus()
        msg.header = self._header()
        msg.supply_no_resource_occupied = False
        msg.supply_resource_occupied = False
        msg.small_energy_status = 0
        msg.big_energy_status = 0
        msg.central_highland = 0  # 未占领
        msg.ladder_highland = 0
        msg.dart_hit_time = 0
        msg.dart_hit_target = 0
        msg.fortress = 0
        msg.outpost_buff = 0
        msg.base_buff = False
        self.pub_field.publish(msg)

    def _pub_team_positions(self):
        msg = RMUCTeamPositions()
        msg.header = self._header()
        msg.hero_x = 1.0
        msg.hero_y = -1.0
        msg.engi_x = 0.5
        msg.engi_y = -0.5
        msg.infantry3_x = 3.0
        msg.infantry3_y = -2.0
        msg.infantry4_x = 4.0
        msg.infantry4_y = -3.0
        self.pub_team_pos.publish(msg)

    def _pub_team_hp(self):
        msg = RMUCTeamHP()
        msg.header = self._header()
        msg.hero_hp = 500
        msg.engi_hp = 400
        msg.infantry3_hp = 500
        msg.infantry4_hp = 500
        msg.sentry_hp = self.args.hp
        msg.outpost_hp = 1500
        msg.base_hp = 5000
        self.pub_team_hp.publish(msg)

    # ────────── 50 Hz 位置 ──────────

    def tick_50hz(self):
        msg = RMUCRobotPosition()
        msg.header = self._header()
        msg.pose_x = 2.0
        msg.pose_y = -2.0
        msg.pose_yaw = 0.0
        msg.is_at_nav_goal = False
        self.pub_position.publish(msg)


def main():
    parser = argparse.ArgumentParser(description="RMUC 2026 裁判系统话题模拟器")
    parser.add_argument("--ns", default="/red_standard_robot1", help="命名空间")
    parser.add_argument("--phase", type=int, default=4, help="比赛阶段 (0-5, 4=比赛中)")
    parser.add_argument("--remain", type=int, default=300, help="阶段剩余时间 (秒)")
    parser.add_argument("--hp", type=int, default=400, help="当前血量")
    parser.add_argument("--ammo", type=int, default=200, help="允许发弹量")
    parser.add_argument("--dead", action="store_true", help="是否战亡")
    args = parser.parse_args()

    rclpy.init()

    # 使用 namespace 让所有话题自动带前缀 (如 /red_standard_robot1/game_status)
    node = RmucTestPublisher(args, namespace=args.ns)

    try:
        node.get_logger().info(
            f"话题命名空间: {args.ns} (话题如 {args.ns}/game_status)"
        )
        node.get_logger().info("Ctrl+C 停止模拟")
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("模拟器停止")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
