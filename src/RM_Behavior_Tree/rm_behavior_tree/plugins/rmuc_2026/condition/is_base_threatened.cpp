#include "rm_behavior_tree/plugins/rmuc_2026/condition/is_base_threatened.hpp"

namespace rm_behavior_tree
{

IsBaseThreatenedCondition::IsBaseThreatenedCondition(
  const std::string & name, const BT::NodeConfig & conf)
: BT::ConditionNode(name, conf) {}

BT::NodeStatus IsBaseThreatenedCondition::tick()
{
  bool threat = false;
  getInput("base_threat", threat);
  return threat ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::IsBaseThreatenedCondition>("IsBaseThreatened");
}
