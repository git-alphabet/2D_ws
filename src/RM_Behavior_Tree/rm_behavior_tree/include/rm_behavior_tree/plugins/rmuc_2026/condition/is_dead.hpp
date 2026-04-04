#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__CONDITION__IS_DEAD_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__CONDITION__IS_DEAD_HPP_

#include <string>
#include <memory>
#include "behaviortree_cpp/condition_node.h"
#include "rm_decision_interfaces/msg/rmuc_robot_status.hpp"

namespace rm_behavior_tree
{

class RmucIsDeadCondition : public BT::SimpleConditionNode
{
public:
  RmucIsDeadCondition(const std::string & name, const BT::NodeConfig & config);

  BT::NodeStatus checkDead();

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::shared_ptr<rm_decision_interfaces::msg::RMUCRobotStatus>>("message")
    };
  }
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__CONDITION__IS_DEAD_HPP_
