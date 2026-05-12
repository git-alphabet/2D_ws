#include "rm_behavior_tree/plugins/rmuc_2026/action/select_objective.hpp"
#include <cstdio>

namespace rm_behavior_tree
{

SelectObjectiveAction::SelectObjectiveAction(
  const std::string & name, const BT::NodeConfig & conf)
: BT::SyncActionNode(name, conf) {}

BT::NodeStatus SelectObjectiveAction::tick()
{
  bool outpost_alive = true;
  int stage_elapsed_time = 0;
  int cap_sustain_time = 0;
  getInput("stage_elapsed_time", stage_elapsed_time);
  getInput("cap_sustain_time", cap_sustain_time);
  getInput("outpost_alive", outpost_alive);

  std::string objective;
  double gx = 0, gy = 0;

  const bool in_cap_window =
    cap_sustain_time > 0 && stage_elapsed_time >= 0 && stage_elapsed_time < cap_sustain_time;

  if (in_cap_window) {
    objective = "CAP_OUTPOST";
    getInput("cap_outpost_x", gx);
    getInput("cap_outpost_y", gy);
  } else if (outpost_alive) {
    objective = "CENTRAL_HIGHLAND";
    getInput("central_highland_x", gx);
    getInput("central_highland_y", gy);
  } else {
    objective = "TRAPEZOIDAL_HIGHLAND";
    getInput("ladder_highland_x", gx);
    getInput("ladder_highland_y", gy);
  }

  static std::string last_obj;
  if (objective != last_obj) {
    fprintf(stderr,
      "[SelectObjective] elapsed=%d, cap_sustain_time=%d, outpost_alive=%d -> %s (%.2f, %.2f)\n",
      stage_elapsed_time, cap_sustain_time, outpost_alive, objective.c_str(), gx, gy);
    last_obj = objective;
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
