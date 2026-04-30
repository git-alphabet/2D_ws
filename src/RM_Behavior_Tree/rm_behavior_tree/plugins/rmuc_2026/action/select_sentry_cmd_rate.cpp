#include "rm_behavior_tree/plugins/rmuc_2026/action/select_sentry_cmd_rate.hpp"

namespace rm_behavior_tree
{

SelectSentryCmdRateAction::SelectSentryCmdRateAction(
  const std::string & name, const BT::NodeConfig & conf)
: BT::SyncActionNode(name, conf)
{
}

BT::NodeStatus SelectSentryCmdRateAction::tick()
{
  bool is_dead = false;
  double alive_hz = 0.2;
  double dead_hz = 1.0;

  getInput("is_dead", is_dead);
  getInput("alive_hz", alive_hz);
  getInput("dead_hz", dead_hz);

  setOutput("hz_out", is_dead ? dead_hz : alive_hz);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::SelectSentryCmdRateAction>("SelectSentryCmdRate");
}