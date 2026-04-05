#include "rm_behavior_tree/plugins/rmuc_2026/action/parse_sentry_blackboard.hpp"

namespace rm_behavior_tree
{

ParseSentryBlackboardAction::ParseSentryBlackboardAction(
  const std::string & name, const BT::NodeConfig & conf)
: BT::SyncActionNode(name, conf)
{
}

BT::NodeStatus ParseSentryBlackboardAction::tick()
{
  // ── 比赛阶段 ──
  auto game_msg = getInput<sp_msgs::msg::RMUCGameStatus>("game_status");
  if (game_msg) {
    setOutput("stage_remain_time", static_cast<int>(game_msg->stage_remain_time));
    // 7 分钟赛制: elapsed = 420 - remain
    setOutput("stage_elapsed_time", 420 - static_cast<int>(game_msg->stage_remain_time));
  }

  // ── 机器人状态 ──
  auto robot_ptr = getInput<std::shared_ptr<sp_msgs::msg::RMUCRobotStatus>>("robot_status");
  if (robot_ptr) {
    const auto & r = **robot_ptr;
    setOutput("hp_cur", static_cast<int>(r.current_hp));
    setOutput("hp_max", static_cast<int>(r.max_hp));
    setOutput("heat_cur", static_cast<int>(r.shooter_heat));
    setOutput("ammo_allow", static_cast<int>(r.ammo_allow));
    setOutput("ammo_left", static_cast<int>(r.ammo_left));
    setOutput("base_hp_cur", static_cast<int>(r.base_hp_cur));
    setOutput("base_hp_max", static_cast<int>(r.base_hp_max));
    setOutput("outpost_alive", r.outpost_alive);
    setOutput("is_dead", r.current_hp <= 0);
    setOutput("is_weak", r.is_weak);

    // 脱战判定
    setOutput("is_disengaged", r.is_disengaged);
    setOutput("disengage_countdown", static_cast<int>(r.disengage_cd_s));

    // 经济状态（由上游电控已计算好）
    setOutput("can_remote_heal", r.can_remote_heal);
    setOutput("can_remote_ammo", r.can_remote_ammo);
    setOutput("team_coins", static_cast<int>(r.team_coins));
  }

  // ── 雷达目标 ──
  auto radar_msg = getInput<sp_msgs::msg::RMUCEnemyTracks>("radar_tracks");
  if (radar_msg && radar_msg->enemy_count > 0) {
    setOutput("has_target", true);
    // 选最近敌方作为 best_target 字符串描述 "id:x:y"
    double px = 0.0, py = 0.0;
    getInput("pose_x", px);
    getInput("pose_y", py);

    double best_dist = 1e9;
    std::string best_target_str = "";
    for (size_t i = 0; i < radar_msg->enemy_count && i < radar_msg->enemy_x.size(); ++i) {
      double dx = radar_msg->enemy_x[i] - px;
      double dy = radar_msg->enemy_y[i] - py;
      double d = std::sqrt(dx * dx + dy * dy);
      if (d < best_dist) {
        best_dist = d;
        best_target_str = std::to_string(radar_msg->enemy_robot_id[i]) + ":" +
                          std::to_string(radar_msg->enemy_x[i]) + ":" +
                          std::to_string(radar_msg->enemy_y[i]);
      }
    }
    setOutput("best_target", best_target_str);

    // 基地威胁判定：任意敌方 y 坐标 ≤ 14m 视为威胁
    bool base_threatened = false;
    for (size_t i = 0; i < radar_msg->enemy_count && i < radar_msg->enemy_y.size(); ++i) {
      if (radar_msg->enemy_y[i] <= 14.0) {
        base_threatened = true;
        break;
      }
    }
    setOutput("base_threat", base_threatened);
    setOutput("fortress_threat", false); // TODO: 结合敌方位置与堡垒距离判定
  } else {
    setOutput("has_target", false);
    setOutput("best_target", std::string(""));
    setOutput("base_threat", false);
    setOutput("fortress_threat", false);
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::ParseSentryBlackboardAction>("ParseSentryBlackboard");
}
