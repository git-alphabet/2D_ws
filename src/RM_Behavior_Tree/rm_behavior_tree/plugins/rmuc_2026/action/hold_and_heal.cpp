#include "rm_behavior_tree/plugins/rmuc_2026/action/hold_and_heal.hpp"

namespace rm_behavior_tree
{

HoldAndHealAction::HoldAndHealAction(
  const std::string & name, const BT::NodeConfig & conf)
: BT::StatefulActionNode(name, conf) {}

BT::NodeStatus HoldAndHealAction::onStart()
{
  return onRunning();
}

BT::NodeStatus HoldAndHealAction::onRunning()
{
  int hp = 0, hp_safe = 280;
  getInput("hp_cur", hp);
  getInput("hp_safe", hp_safe);

  if (hp >= hp_safe) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

void HoldAndHealAction::onHalted() {}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::HoldAndHealAction>("HoldAndHeal");
}
