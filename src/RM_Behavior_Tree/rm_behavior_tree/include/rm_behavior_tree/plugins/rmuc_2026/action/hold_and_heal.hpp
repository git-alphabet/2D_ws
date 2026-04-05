#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__HOLD_AND_HEAL_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__HOLD_AND_HEAL_HPP_

#include <string>
#include "behaviortree_cpp/action_node.h"

namespace rm_behavior_tree
{
/// 在补血区驻留等待血量恢复到安全线。RUNNING 直到 hp ≥ hp_safe 或脱战恢复
class HoldAndHealAction : public BT::StatefulActionNode
{
public:
  HoldAndHealAction(const std::string & name, const BT::NodeConfig & conf);
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("hp_cur"),
      BT::InputPort<int>("hp_max"),
      BT::InputPort<int>("hp_safe", "280", "hp_safe"),
      BT::InputPort<int>("stage_elapsed_time")};
  }
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
};
}  // namespace rm_behavior_tree
#endif
