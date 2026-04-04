#include "rm_behavior_tree/plugins/rmuc_2026/action/select_nearest_dispel_card.hpp"
#include <array>

namespace rm_behavior_tree
{

SelectNearestDispelCardAction::SelectNearestDispelCardAction(
  const std::string & name, const BT::NodeConfig & conf)
: BT::SyncActionNode(name, conf) {}

BT::NodeStatus SelectNearestDispelCardAction::tick()
{
  double px = 0, py = 0;
  getInput("pose_x", px);
  getInput("pose_y", py);

  struct Pt { double x; double y; };
  std::array<Pt, 3> pts{};
  double v = 0;
  getInput("supply_x", v); pts[0].x = v;
  getInput("supply_y", v); pts[0].y = v;
  getInput("base_buff_x", v); pts[1].x = v;
  getInput("base_buff_y", v); pts[1].y = v;
  getInput("outpost_buff_x", v); pts[2].x = v;
  getInput("outpost_buff_y", v); pts[2].y = v;

  double best = 1e9;
  Pt best_pt{0, 0};
  for (auto & p : pts) {
    double d = std::hypot(p.x - px, p.y - py);
    if (d < best) { best = d; best_pt = p; }
  }
  setOutput("goal_x", best_pt.x);
  setOutput("goal_y", best_pt.y);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::SelectNearestDispelCardAction>("SelectNearestDispelCard");
}
