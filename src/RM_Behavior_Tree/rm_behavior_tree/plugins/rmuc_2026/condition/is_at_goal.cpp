#include "rm_behavior_tree/plugins/rmuc_2026/condition/is_at_goal.hpp"

namespace rm_behavior_tree
{

IsAtGoalCondition::IsAtGoalCondition(
  const std::string & name, const BT::NodeConfig & conf)
: BT::ConditionNode(name, conf) {}

BT::NodeStatus IsAtGoalCondition::tick()
{
  double px = 0, py = 0, gx = 0, gy = 0, radius = 0.35;
  getInput("pose_x", px);
  getInput("pose_y", py);
  getInput("goal_x", gx);
  getInput("goal_y", gy);
  getInput("arrive_radius", radius);

  return (std::hypot(gx - px, gy - py) < radius)
    ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::IsAtGoalCondition>("IsAtGoal");
}
