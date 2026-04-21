#include "rm_behavior_tree/plugins/rmuc_2026/action/parse_sentry_blackboard.hpp"
#include <iostream>

namespace rm_behavior_tree
{

namespace
{
constexpr double kBaseThreatEnterDistance = 5.0;
constexpr int64_t kBaseThreatCalmTimeoutMs = 30000;  // 30秒无威胁自动解除
}

ParseSentryBlackboardAction::ParseSentryBlackboardAction(
  const std::string & name, const BT::NodeConfig & conf)
: BT::SyncActionNode(name, conf)
{
}

BT::NodeStatus ParseSentryBlackboardAction::tick()
{
  // ── 比赛阶段 (GameStatus) ──
  auto game_msg = getInput<sp_msgs::msg::RMUCGameStatus>("game_status");
  if (game_msg) {
    setOutput("stage_remain_time", static_cast<int>(game_msg->stage_remain_time));
    setOutput("stage_elapsed_time", 420 - static_cast<int>(game_msg->stage_remain_time));
  }

  // ── 机器人状态 (RobotStatus) ──
  auto robot_ptr = getInput<std::shared_ptr<sp_msgs::msg::RMUCRobotStatus>>("robot_status");
  if (robot_ptr) {
    const auto & r = **robot_ptr;
    setOutput("hp_cur", static_cast<int>(r.current_hp));
    setOutput("ammo_allow", static_cast<int>(r.ammo_allow));
    // base_hp_cur: 已改由 team_hp.base_hp 接管 (0x0003 offset 14)
    // outpost_alive 不再使用 robot_status.outpost_alive (bool转换可能有误)
    // 将由 team_hp.outpost_hp > 0 得出（0x0003 offset 12 原始 uint16_t)
    setOutput("is_dead", r.current_hp <= 0);
    setOutput("has_target", r.is_detect_enemy);
  }

  auto radar_tracks = getInput<sp_msgs::msg::RMUCEnemyTracks>("radar_tracks");
  // 注意: 雷达仅用于基地威胁判断，不参与 has_target (战斗仅依赖云台视觉 is_detect_enemy)

  // ── 队伍血量 (TeamHP) ──
  auto th = getInput<sp_msgs::msg::RMUCTeamHP>("team_hp");
  if (th) {
    // 引用原始 uint16_t 字段 (0x0003 offset 12/14)，避免 robot_status bool转换失真
    setOutput("outpost_alive", th->outpost_hp > 0);
    setOutput("base_hp_cur", static_cast<int>(th->base_hp));  // 0x0003 offset 14 接管
  }

  // 基地危机锁存：
  // 进入条件：雷达扫描到敌人在 defend_anchor 5m 内 + 基地 HP 下降
  // 退出条件：云台没有扫描到敌人 + 基地不掉血，持续 30 秒自动解除
  bool base_threat = base_threat_latched_;
  double defend_anchor_x = 0.0;
  double defend_anchor_y = 0.0;
  getInput("defend_anchor_x", defend_anchor_x);
  getInput("defend_anchor_y", defend_anchor_y);

  // 雷达距离计算（用于进入条件）
  bool any_enemy_near = false;
  if (radar_tracks &&
    radar_tracks->enemy_x.size() == radar_tracks->enemy_y.size() &&
    !radar_tracks->enemy_x.empty())
  {
    for (size_t index = 0; index < radar_tracks->enemy_x.size(); ++index) {
      const double dx = static_cast<double>(radar_tracks->enemy_x[index]) - defend_anchor_x;
      const double dy = static_cast<double>(radar_tracks->enemy_y[index]) - defend_anchor_y;
      const double distance = std::hypot(dx, dy);
      if (distance < kBaseThreatEnterDistance) {
        any_enemy_near = true;
        break;
      }
    }
  }

  // 进入条件：雷达扫到敌人在基地附近 + 基地掉血
  bool base_hp_is_dropping = false;
  if (th) {
    base_hp_is_dropping = (last_base_hp_ >= 0 && th->base_hp < static_cast<uint16_t>(last_base_hp_));
    if (base_hp_is_dropping && any_enemy_near) {
      base_threat = true;
      std::cout << "[ParseSentryBlackboard] 基地威胁触发：雷达扫到敌人在基地 5m 内且基地掉血"
                << std::endl;
    }
    last_base_hp_ = static_cast<int>(th->base_hp);
  }

  // 退出条件：云台没扫到敌人 + 基地不掉血，持续 30 秒自动解除
  bool gimbal_detects_enemy = false;
  if (robot_ptr) {
    gimbal_detects_enemy = (**robot_ptr).is_detect_enemy;
  }

  if (base_threat) {
    bool is_calm = !gimbal_detects_enemy && !base_hp_is_dropping;
    auto now = std::chrono::steady_clock::now();
    if (is_calm) {
      if (!base_threat_calm_tracking_) {
        base_threat_calm_tracking_ = true;
        base_threat_calm_start_ = now;
      } else {
        auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
          now - base_threat_calm_start_).count();
        if (elapsed_ms >= kBaseThreatCalmTimeoutMs) {
          base_threat = false;
          base_threat_calm_tracking_ = false;
          std::cout << "[ParseSentryBlackboard] 基地威胁解除：连续 30s 云台未检测到敌人且基地不掉血"
                    << std::endl;
        }
      }
    } else {
      // 云台检测到敌人或基地在掉血 → 重置平静计时器
      base_threat_calm_tracking_ = false;
    }
  } else {
    base_threat_calm_tracking_ = false;
  }

  base_threat_latched_ = base_threat;
  setOutput("base_threat", base_threat);

  // ── 脱战检测 ──
  // 规则: 存活状态下连续 6 秒未发射弹丸且未被扣血 = 脱战。
  // 开局默认脱战。
  std::uint64_t now_ms = 0;
  getInput("now_ms", now_ms);

  if (robot_ptr) {
    const auto & rd = **robot_ptr;
    if (!disengage_initialized_) {
      last_shooter_heat_ = rd.shooter_heat;
      last_current_hp_ = rd.current_hp;
      last_activity_ms_ = 0;  // 开局视为脱战(activity=0 → elapsed > 6s 立即成立)
      disengage_initialized_ = true;
    }

    const bool fired = (rd.shooter_heat > last_shooter_heat_);
    const bool took_damage = (rd.current_hp < last_current_hp_ && last_current_hp_ > 0);

    if (fired || took_damage || rd.current_hp <= 0) {
      last_activity_ms_ = now_ms;
    }
    last_shooter_heat_ = rd.shooter_heat;
    last_current_hp_ = rd.current_hp;

    const bool is_disengaged = (rd.current_hp > 0 && now_ms > 0 &&
      (now_ms - last_activity_ms_) >= 6000);
    setOutput("is_disengaged", is_disengaged);
  } else {
    setOutput("is_disengaged", false);
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::ParseSentryBlackboardAction>("ParseSentryBlackboard");
}
