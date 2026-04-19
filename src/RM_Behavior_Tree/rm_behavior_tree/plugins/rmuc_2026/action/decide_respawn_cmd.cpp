#include "rm_behavior_tree/plugins/rmuc_2026/action/decide_respawn_cmd.hpp"

namespace rm_behavior_tree
{

DecideRespawnCmdAction::DecideRespawnCmdAction(
  const std::string & name, const BT::NodeConfig & conf)
: BT::SyncActionNode(name, conf)
{
}

BT::NodeStatus DecideRespawnCmdAction::tick()
{
  bool is_dead = false;
  int coins = 0, remain_s = 420, base_hp = 5000;
  int instant_cost = 0;

  getInput("is_dead", is_dead);
  getInput("team_coins", coins);
  getInput("stage_remain_time", remain_s);
  getInput("base_hp_cur", base_hp);
  getInput("instant_respawn_cost", instant_cost);

  int confirm = 0, instant = 0;

  if (is_dead) {
    // 始终确认普通复活
    confirm = 1;

    // 立即复活条件：
    // 1. 基地血量 < 2500 (满血5000的50%) 且 金币 ≥ 兑换费用 * 1.7
    // 2. 或 剩余时间 < 60s
    bool base_critical = (base_hp < 2500);
    bool afford = (instant_cost > 0 && coins >= static_cast<int>(instant_cost * 1.7));
    if ((base_critical && afford) || remain_s < 60) {
      instant = 1;
    }
  }

  setOutput("confirm_respawn", confirm);
  setOutput("confirm_instant_respawn", instant);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::DecideRespawnCmdAction>("DecideRespawnCmd");
}
