#include "rm_behavior_tree/plugins/rmuc_2026/action/select_safe_retreat_goal.hpp"

namespace rm_behavior_tree
{

SelectSafeRetreatGoalAction::SelectSafeRetreatGoalAction(
  const std::string & name, const BT::NodeConfig & conf)
: BT::SyncActionNode(name, conf) {}

BT::NodeStatus SelectSafeRetreatGoalAction::tick()
{
  double px = 0, py = 0, sx = 0, sy = 0, dx = 0, dy = 0;
  getInput("pose_x", px);
  getInput("pose_y", py);
  getInput("supply_x", sx);
  getInput("supply_y", sy);
  getInput("defend_anchor_x", dx);
  getInput("defend_anchor_y", dy);

  double d_supply = std::hypot(sx - px, sy - py);
  double d_defend = std::hypot(dx - px, dy - py);

  if (d_supply <= d_defend) {
    setOutput("goal_x", sx);
    setOutput("goal_y", sy);
  } else {
    setOutput("goal_x", dx);
    setOutput("goal_y", dy);
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::SelectSafeRetreatGoalAction>("SelectSafeRetreatGoal");
}
