#include "rm_behavior_tree/plugins/rmuc_2026/condition/is_at_goal.hpp"
#include <iostream>
#include <chrono>

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

  double dist = std::hypot(gx - px, gy - py);
  bool at_goal = dist < radius;

  // 仅在状态变化时打印一次
  static bool last_at_goal = false;
  static bool first_print = true;
  if (first_print || at_goal != last_at_goal) {
    first_print = false;
    last_at_goal = at_goal;
    std::cout << "[IsAtGoal:" << name() << "] pose=(" << px << "," << py
              << ") goal=(" << gx << "," << gy
              << ") dist=" << dist << " radius=" << radius
              << " → " << (at_goal ? "SUCCESS" : "FAILURE") << std::endl;
  }

  return at_goal ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::IsAtGoalCondition>("IsAtGoal");
}
