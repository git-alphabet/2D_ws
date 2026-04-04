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
  int coins = 0, remain_s = 420, base_hp = 5000, base_max = 5000;
  bool base_threat = false;

  getInput("is_dead", is_dead);
  getInput("team_coins", coins);
  getInput("stage_remain_time", remain_s);
  getInput("base_hp_cur", base_hp);
  getInput("base_hp_max", base_max);
  getInput("base_threat", base_threat);

  int confirm = 0, instant = 0;

  if (is_dead) {
    // 始终确认普通复活
    confirm = 1;
    // 兑换立即复活条件：基地受威胁且金币足够(≥300) 或 剩余时间<60s
    if ((base_threat && coins >= 300) || remain_s < 60) {
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
