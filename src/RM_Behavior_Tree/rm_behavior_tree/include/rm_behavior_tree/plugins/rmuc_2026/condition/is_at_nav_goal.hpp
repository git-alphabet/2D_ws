#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__CONDITION__IS_AT_NAV_GOAL_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__CONDITION__IS_AT_NAV_GOAL_HPP_

#include <string>

#include "behaviortree_cpp/condition_node.h"

namespace rm_behavior_tree
{

class RmucIsAtNavGoalCondition : public BT::ConditionNode
{
public:
  RmucIsAtNavGoalCondition(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<bool>("is_at_nav_goal")
    };
  }

  BT::NodeStatus tick() override;
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__CONDITION__IS_AT_NAV_GOAL_HPP_
