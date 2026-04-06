#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__HOLD_OBJECTIVE_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__HOLD_OBJECTIVE_HPP_

#include <string>
#include <chrono>
#include "behaviortree_cpp/action_node.h"

namespace rm_behavior_tree
{
/// 在目标点驻留指定时间。如果期间基地受威胁或有目标则立即中断
class HoldObjectiveAction : public BT::StatefulActionNode
{
public:
  HoldObjectiveAction(const std::string & name, const BT::NodeConfig & conf);
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("hold_ms", "12000", "hold_ms"),
      BT::InputPort<bool>("base_threat"),
      BT::InputPort<bool>("has_target")};
  }
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::chrono::steady_clock::time_point start_;
  int hold_ms_{12000};
};
}  // namespace rm_behavior_tree
#endif
