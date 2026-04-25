#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__CONDITION__IS_HP_BELOW_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__CONDITION__IS_HP_BELOW_HPP_

#include <string>
#include <memory>
#include "behaviortree_cpp/condition_node.h"
#include "sp_msgs/msg/rmuc_robot_status.hpp"

namespace rm_behavior_tree
{
class RmucIsHPBelowCondition : public BT::SimpleConditionNode
{
public:
  RmucIsHPBelowCondition(const std::string & name, const BT::NodeConfig & config);
  BT::NodeStatus checkHPBelow();

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::shared_ptr<sp_msgs::msg::RMUCRobotStatus>>("message"),
      BT::InputPort<int>("hp_threshold")};
  }
};
}  // namespace rm_behavior_tree

#endif
