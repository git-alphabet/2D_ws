#include "rm_behavior_tree/plugins/rmuc_2026/action/hold_objective.hpp"

namespace rm_behavior_tree
{

HoldObjectiveAction::HoldObjectiveAction(
  const std::string & name, const BT::NodeConfig & conf)
: BT::StatefulActionNode(name, conf) {}

BT::NodeStatus HoldObjectiveAction::onStart()
{
  getInput("hold_ms", hold_ms_);
  start_ = std::chrono::steady_clock::now();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus HoldObjectiveAction::onRunning()
{
  bool threat = false, has_tgt = false;
  getInput("base_threat", threat);
  getInput("has_target", has_tgt);

  // 高优先级事件中断驻留
  if (threat || has_tgt) {
    return BT::NodeStatus::SUCCESS;
  }

  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - start_).count();

  return (elapsed >= hold_ms_) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}

void HoldObjectiveAction::onHalted() {}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::HoldObjectiveAction>("HoldObjective");
}
