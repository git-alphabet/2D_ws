#include "rm_behavior_tree/plugins/rmuc_2026/action/select_nearest_resupply_station.hpp"
#include <array>
#include <utility>

namespace rm_behavior_tree
{

SelectNearestResupplyStationAction::SelectNearestResupplyStationAction(
  const std::string & name, const BT::NodeConfig & conf)
: BT::SyncActionNode(name, conf) {}

BT::NodeStatus SelectNearestResupplyStationAction::tick()
{
  double px = 0, py = 0;
  getInput("pose_x", px);
  getInput("pose_y", py);

  bool outpost_alive = true;
  getInput("outpost_alive", outpost_alive);

  struct Pt { double x; double y; };
  double v = 0;

  Pt supply{};
  getInput("supply_zone_x", v); supply.x = v;
  getInput("supply_zone_y", v); supply.y = v;

  Pt base_buff{};
  getInput("base_buff_x", v); base_buff.x = v;
  getInput("base_buff_y", v); base_buff.y = v;

  Pt outpost_buff{};
  getInput("outpost_buff_x", v); outpost_buff.x = v;
  getInput("outpost_buff_y", v); outpost_buff.y = v;

  std::array<Pt, 3> pts{supply, base_buff, outpost_buff};
  const int count = outpost_alive ? 3 : 2;

  double best = 1e9;
  Pt best_pt{supply.x, supply.y};
  for (int i = 0; i < count; ++i) {
    double d = std::hypot(pts[i].x - px, pts[i].y - py);
    if (d < best) { best = d; best_pt = pts[i]; }
  }
  setOutput("goal_x", best_pt.x);
  setOutput("goal_y", best_pt.y);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::SelectNearestResupplyStationAction>("SelectNearestResupplyStation");
}
