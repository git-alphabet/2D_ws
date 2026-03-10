#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__CONDITION__IS_FIRE_WINDOW_OK_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__CONDITION__IS_FIRE_WINDOW_OK_HPP_

#include <string>
#include "behaviortree_cpp/condition_node.h"

namespace rm_behavior_tree
{
/// 射击窗口：热量 < 上限 且 弹丸配额 > 0 且 非虚弱
class IsFireWindowOkCondition : public BT::ConditionNode
{
public:
  IsFireWindowOkCondition(const std::string & name, const BT::NodeConfig & conf);
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("heat_cur"),
      BT::InputPort<int>("heat_high", "210", "heat_high"),
      BT::InputPort<int>("ammo_allow"),
      BT::InputPort<bool>("is_weak")};
  }
  BT::NodeStatus tick() override;
};
}  // namespace rm_behavior_tree
#endif
