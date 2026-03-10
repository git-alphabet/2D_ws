#include "rm_behavior_tree/plugins/rmuc_2026/condition/is_at_nav_goal.hpp"

namespace rm_behavior_tree
{

RmucIsAtNavGoalCondition::RmucIsAtNavGoalCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::ConditionNode(name, config)
{
}

BT::NodeStatus RmucIsAtNavGoalCondition::tick()
{
  auto res = getInput<bool>("is_at_nav_goal");
  if (!res) {
    return BT::NodeStatus::FAILURE;
  }
  return res.value() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::RmucIsAtNavGoalCondition>("RmucIsAtNavGoal");
}
