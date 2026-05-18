#include "rm_behavior_tree/plugins/rmuc_2026/condition/is_semantic_zone_active.hpp"

namespace rm_behavior_tree
{

IsSemanticZoneActiveCondition::IsSemanticZoneActiveCondition(
  const std::string & name, const BT::NodeConfig & conf)
: BT::ConditionNode(name, conf) {}

BT::NodeStatus IsSemanticZoneActiveCondition::tick()
{
  bool active = false;
  getInput("semantic_zone_active", active);
  return active ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::IsSemanticZoneActiveCondition>(
    "IsSemanticZoneActive");
}
