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

    // 经济状态（由上游电控已计算好）
    setOutput("can_remote_heal", r.can_remote_heal);
    setOutput("can_remote_ammo", r.can_remote_ammo);
    setOutput("team_coins", static_cast<int>(r.team_coins));

    // 视觉检测敌人 → has_target
    setOutput("has_target", r.is_detect_enemy);
  }

  // base_threat / fortress_threat 由 base HP 判定兜底（IsBaseThreatened 检查 hp < 50%）
  setOutput("base_threat", false);
  setOutput("fortress_threat", false);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::ParseSentryBlackboardAction>("ParseSentryBlackboard");
}
