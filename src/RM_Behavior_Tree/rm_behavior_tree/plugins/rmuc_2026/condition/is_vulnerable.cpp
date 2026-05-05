#include "rm_behavior_tree/plugins/rmuc_2026/condition/is_vulnerable.hpp"

namespace rm_behavior_tree
{

IsVulnerableCondition::IsVulnerableCondition(
  const std::string & name, const BT::NodeConfig & conf)
: BT::ConditionNode(name, conf)
{
}

BT::NodeStatus IsVulnerableCondition::tick()
{
  auto robot_buff = getInput<sp_msgs::msg::RMUCRobotBuff>("robot_buff");
  int min_vulnerability_pct = 1;
  getInput("min_vulnerability_pct", min_vulnerability_pct);

  if (!robot_buff) {
    return BT::NodeStatus::FAILURE;
  }

  return robot_buff->vulnerability_pct >= min_vulnerability_pct ?
         BT::NodeStatus::SUCCESS :
         BT::NodeStatus::FAILURE;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::IsVulnerableCondition>("IsVulnerable");
}
