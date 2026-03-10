#include "rm_behavior_tree/plugins/rmul_2026/action/keep_running.hpp"

namespace rm_behavior_tree
{

KeepRunningAction::KeepRunningAction(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
}

BT::NodeStatus KeepRunningAction::onStart()
{
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus KeepRunningAction::onRunning()
{
  return BT::NodeStatus::RUNNING;
}

void KeepRunningAction::onHalted()
{
  // nothing to do
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::KeepRunningAction>("KeepRunning");
}
