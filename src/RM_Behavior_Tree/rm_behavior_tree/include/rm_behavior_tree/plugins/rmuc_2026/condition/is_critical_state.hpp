#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__CONDITION__IS_CRITICAL_STATE_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__CONDITION__IS_CRITICAL_STATE_HPP_

#include <string>
#include "behaviortree_cpp/condition_node.h"

namespace rm_behavior_tree
{
/// HP < hp_critical 或 heat > heat_critical 时返回 SUCCESS
class IsCriticalStateCondition : public BT::ConditionNode
{
public:
  IsCriticalStateCondition(const std::string & name, const BT::NodeConfig & conf);
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("hp_cur"),
      BT::InputPort<int>("hp_critical", "80", "hp_critical"),
      BT::InputPort<int>("heat_cur"),
      BT::InputPort<int>("heat_critical", "245", "heat_critical")};
  }
  BT::NodeStatus tick() override;
};
}  // namespace rm_behavior_tree
#endif
