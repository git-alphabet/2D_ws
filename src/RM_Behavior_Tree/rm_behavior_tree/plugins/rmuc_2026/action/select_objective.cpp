#include "rm_behavior_tree/plugins/rmuc_2026/action/select_objective.hpp"

namespace rm_behavior_tree
{

SelectObjectiveAction::SelectObjectiveAction(
  const std::string & name, const BT::NodeConfig & conf)
: BT::SyncActionNode(name, conf) {}

BT::NodeStatus SelectObjectiveAction::tick()
{
  bool outpost_alive = true;
  getInput("outpost_alive", outpost_alive);

  std::string objective;
  double gx = 0, gy = 0;

  if (outpost_alive) {
    objective = "CENTRAL_HIGHLAND";
    getInput("central_highland_x", gx);
    getInput("central_highland_y", gy);
  } else {
    objective = "TRAPEZOIDAL_HIGHLAND";
    getInput("ladder_highland_x", gx);
    getInput("ladder_highland_y", gy);
  }

  setOutput("goal_x", gx);
  setOutput("goal_y", gy);
  setOutput("objective_name", objective);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::SelectObjectiveAction>("SelectObjective");
}
