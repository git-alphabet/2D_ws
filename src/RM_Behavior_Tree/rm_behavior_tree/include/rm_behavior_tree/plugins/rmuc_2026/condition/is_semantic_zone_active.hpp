#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__CONDITION__IS_SEMANTIC_ZONE_ACTIVE_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__CONDITION__IS_SEMANTIC_ZONE_ACTIVE_HPP_

#include <string>
#include "behaviortree_cpp/condition_node.h"

namespace rm_behavior_tree
{
/// 语义区激活判定：统一消费 ParseSentryBlackboard 派生后的 semantic_zone_active
class IsSemanticZoneActiveCondition : public BT::ConditionNode
{
public:
  IsSemanticZoneActiveCondition(const std::string & name, const BT::NodeConfig & conf);
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<bool>("semantic_zone_active", false, "语义区是否激活")};
  }
  BT::NodeStatus tick() override;
};
}  // namespace rm_behavior_tree
#endif
