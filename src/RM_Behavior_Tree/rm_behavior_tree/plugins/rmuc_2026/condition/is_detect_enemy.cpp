#include "rm_behavior_tree/plugins/rmuc_2026/condition/is_detect_enemy.hpp"

#include <functional>

namespace rm_behavior_tree
{

RmucIsDetectEnemyCondition::RmucIsDetectEnemyCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(
    name, std::bind(&RmucIsDetectEnemyCondition::checkDetectEnemy, this), config)
{
}

BT::NodeStatus RmucIsDetectEnemyCondition::checkDetectEnemy()
{
  auto msg = getInput<std::shared_ptr<sp_msgs::msg::RMUCRobotStatus>>("message");

  if (!msg) {
    return BT::NodeStatus::FAILURE;
  }

  return (*msg)->is_detect_enemy ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::RmucIsDetectEnemyCondition>("RmucIsDetectEnemy");
}
