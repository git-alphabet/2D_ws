#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__CONDITION__IS_AMMO_BELOW_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__CONDITION__IS_AMMO_BELOW_HPP_

#include <string>
#include "behaviortree_cpp/condition_node.h"

namespace rm_behavior_tree
{
class IsAmmoBelowCondition : public BT::ConditionNode
{
public:
  IsAmmoBelowCondition(const std::string & name, const BT::NodeConfig & conf);
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("ammo_allow"),
      BT::InputPort<int>("ammo_low", "80", "ammo_low")};
  }
  BT::NodeStatus tick() override;
};
}  // namespace rm_behavior_tree
#endif
