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
  getInput("is_dead", is_dead);

  // 死亡时始终确认免费复活（等读条结束），不买活
  setOutput("confirm_respawn", is_dead ? 1 : 0);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::DecideRespawnCmdAction>("DecideRespawnCmd");
}
