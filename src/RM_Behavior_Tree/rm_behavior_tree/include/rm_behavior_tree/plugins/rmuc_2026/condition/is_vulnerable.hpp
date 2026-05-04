#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__CONDITION__IS_VULNERABLE_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__CONDITION__IS_VULNERABLE_HPP_

#include <string>
#include "behaviortree_cpp/condition_node.h"
#include "sp_msgs/msg/rmuc_robot_buff.hpp"

namespace rm_behavior_tree
{
class IsVulnerableCondition : public BT::ConditionNode
{
public:
  IsVulnerableCondition(const std::string & name, const BT::NodeConfig & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<sp_msgs::msg::RMUCRobotBuff>("robot_buff"),
      BT::InputPort<int>("min_vulnerability_pct", "1", "minimum vulnerability percent")};
  }

  BT::NodeStatus tick() override;
};
}  // namespace rm_behavior_tree

#endif
