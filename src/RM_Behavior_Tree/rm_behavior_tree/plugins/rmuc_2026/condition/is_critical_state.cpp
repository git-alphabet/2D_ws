#include "rm_behavior_tree/plugins/rmuc_2026/condition/is_critical_state.hpp"

namespace rm_behavior_tree
{

IsCriticalStateCondition::IsCriticalStateCondition(
  const std::string & name, const BT::NodeConfig & conf)
: BT::ConditionNode(name, conf) {}

BT::NodeStatus IsCriticalStateCondition::tick()
{
  int hp = 400, hp_crit = 80, heat = 0, heat_crit = 245;
  getInput("hp_cur", hp);
  getInput("hp_critical", hp_crit);
  getInput("heat_cur", heat);
  getInput("heat_critical", heat_crit);

  return (hp < hp_crit || heat > heat_crit) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::IsCriticalStateCondition>("IsCriticalState");
}
