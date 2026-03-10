#include "rm_behavior_tree/plugins/rmuc_2026/action/waypoint_patrol.hpp"
#include <array>

namespace rm_behavior_tree
{

WaypointPatrolAction::WaypointPatrolAction(
  const std::string & name, const BT::NodeConfig & conf)
: BT::SyncActionNode(name, conf) {}

BT::NodeStatus WaypointPatrolAction::tick()
{
  double px = 0, py = 0;
  getInput("pose_x", px);
  getInput("pose_y", py);

  struct Pt { double x; double y; };
  std::array<Pt, 3> wpts{};
  double v = 0;
  getInput("wpt0_x", v); wpts[0].x = v;
  getInput("wpt0_y", v); wpts[0].y = v;
  getInput("wpt1_x", v); wpts[1].x = v;
  getInput("wpt1_y", v); wpts[1].y = v;
  getInput("wpt2_x", v); wpts[2].x = v;
  getInput("wpt2_y", v); wpts[2].y = v;

  // 到达当前目标附近（< 0.5m）时切换下一个
  double d = std::hypot(wpts[current_idx_].x - px, wpts[current_idx_].y - py);
  if (d < 0.5) {
    current_idx_ = (current_idx_ + 1) % 3;
  }

  setOutput("goal_x", wpts[current_idx_].x);
  setOutput("goal_y", wpts[current_idx_].y);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::WaypointPatrolAction>("WaypointPatrol");
}
