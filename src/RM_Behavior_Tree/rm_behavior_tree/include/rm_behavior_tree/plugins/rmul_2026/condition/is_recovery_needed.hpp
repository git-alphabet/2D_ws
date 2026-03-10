#ifndef RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_RECOVERY_NEEDED_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_RECOVERY_NEEDED_HPP_

#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"

namespace rm_behavior_tree
{

class IsRecoveryNeededCondition : public BT::ConditionNode
{
public:
  IsRecoveryNeededCondition(
    const std::string & name,
    const BT::NodeConfig & conf,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<bool>("need_recovery")
    };
  }

  BT::NodeStatus tick() override;

private:
  BT::RosNodeParams params_;
};

}  // namespace rm_behavior_tree

#endif
