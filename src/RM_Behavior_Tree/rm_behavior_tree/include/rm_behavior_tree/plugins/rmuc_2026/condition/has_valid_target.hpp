#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__CONDITION__HAS_VALID_TARGET_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__CONDITION__HAS_VALID_TARGET_HPP_

#include <string>
#include "behaviortree_cpp/condition_node.h"

namespace rm_behavior_tree
{
class HasValidTargetCondition : public BT::ConditionNode
{
public:
  HasValidTargetCondition(const std::string & name, const BT::NodeConfig & conf);
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<bool>("has_target"),
      BT::InputPort<std::string>("best_target")};
  }
  BT::NodeStatus tick() override;
};
}  // namespace rm_behavior_tree
#endif
