#include "rm_behavior_tree/plugins/rmuc_2026/condition/is_base_threatened.hpp"

namespace rm_behavior_tree
{

IsBaseThreatenedCondition::IsBaseThreatenedCondition(
  const std::string & name, const BT::NodeConfig & conf)
: BT::ConditionNode(name, conf) {}

BT::NodeStatus IsBaseThreatenedCondition::tick()
{
  bool threat = false;
  int hp = 5000, hp_max = 5000;
  getInput("base_threat", threat);
  getInput("base_hp_cur", hp);
  getInput("base_hp_max", hp_max);

  if (threat) return BT::NodeStatus::SUCCESS;
  if (hp_max > 0 && hp < hp_max / 2) return BT::NodeStatus::SUCCESS;
  return BT::NodeStatus::FAILURE;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::IsBaseThreatenedCondition>("IsBaseThreatened");
}
