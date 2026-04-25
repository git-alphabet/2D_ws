#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__CONDITION__IS_BASE_THREATENED_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__CONDITION__IS_BASE_THREATENED_HPP_

#include <string>
#include "behaviortree_cpp/condition_node.h"

namespace rm_behavior_tree
{
/// 基地受威胁判定：统一消费黑板派生后的 base_threat
class IsBaseThreatenedCondition : public BT::ConditionNode
{
public:
  IsBaseThreatenedCondition(const std::string & name, const BT::NodeConfig & conf);
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<bool>("base_threat")};
  }
  BT::NodeStatus tick() override;
};
}  // namespace rm_behavior_tree
#endif
