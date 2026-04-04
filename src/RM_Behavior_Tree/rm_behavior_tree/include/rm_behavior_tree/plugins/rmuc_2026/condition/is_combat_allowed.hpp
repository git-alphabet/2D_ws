#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__CONDITION__IS_COMBAT_ALLOWED_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__CONDITION__IS_COMBAT_ALLOWED_HPP_

#include <string>
#include "behaviortree_cpp/condition_node.h"

namespace rm_behavior_tree
{
/// 战斗允许条件：非虚弱 且 弹丸配额>0 且 热量<上限 且 HP>安全线
class IsCombatAllowedCondition : public BT::ConditionNode
{
public:
  IsCombatAllowedCondition(const std::string & name, const BT::NodeConfig & conf);
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<bool>("is_weak"),
      BT::InputPort<int>("ammo_allow"),
      BT::InputPort<int>("heat_cur"),
      BT::InputPort<int>("heat_high", "210", "heat_high"),
      BT::InputPort<int>("hp_cur"),
      BT::InputPort<int>("hp_low", "180", "hp_low")};
  }
  BT::NodeStatus tick() override;
};
}  // namespace rm_behavior_tree
#endif
