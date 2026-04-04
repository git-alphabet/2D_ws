#include "rm_behavior_tree/plugins/rmuc_2026/condition/is_hp_below.hpp"
#include <functional>

namespace rm_behavior_tree
{

RmucIsHPBelowCondition::RmucIsHPBelowCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&RmucIsHPBelowCondition::checkHPBelow, this), config)
{
}

BT::NodeStatus RmucIsHPBelowCondition::checkHPBelow()
{
  int hp_threshold = 0;
  auto msg = getInput<std::shared_ptr<rm_decision_interfaces::msg::RMUCRobotStatus>>("message");
  getInput("hp_threshold", hp_threshold);
  if (!msg) {
    return BT::NodeStatus::FAILURE;
  }
  if ((*msg)->current_hp < hp_threshold) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::RmucIsHPBelowCondition>("RmucIsHPBelow");
}
