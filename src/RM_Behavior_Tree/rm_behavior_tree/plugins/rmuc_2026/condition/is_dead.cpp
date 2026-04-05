#include "rm_behavior_tree/plugins/rmuc_2026/condition/is_dead.hpp"

#include <functional>

namespace rm_behavior_tree
{

RmucIsDeadCondition::RmucIsDeadCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&RmucIsDeadCondition::checkDead, this), config)
{
}

BT::NodeStatus RmucIsDeadCondition::checkDead()
{
  auto msg = getInput<std::shared_ptr<sp_msgs::msg::RMUCRobotStatus>>("message");

  if (!msg) {
    return BT::NodeStatus::FAILURE;
  }

  if ((*msg)->current_hp <= 0) {
    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::RmucIsDeadCondition>("RmucIsDead");
}
