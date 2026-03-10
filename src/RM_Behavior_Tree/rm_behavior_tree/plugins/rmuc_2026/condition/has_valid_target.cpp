#include "rm_behavior_tree/plugins/rmuc_2026/condition/has_valid_target.hpp"

namespace rm_behavior_tree
{

HasValidTargetCondition::HasValidTargetCondition(
  const std::string & name, const BT::NodeConfig & conf)
: BT::ConditionNode(name, conf) {}

BT::NodeStatus HasValidTargetCondition::tick()
{
  bool has = false;
  std::string target;
  getInput("has_target", has);
  getInput("best_target", target);
  return (has && !target.empty()) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::HasValidTargetCondition>("HasValidTarget");
}
