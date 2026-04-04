#include "rm_behavior_tree/plugins/rmuc_2026/action/select_best_target.hpp"

namespace rm_behavior_tree
{

SelectBestTargetAction::SelectBestTargetAction(
  const std::string & name, const BT::NodeConfig & conf)
: BT::SyncActionNode(name, conf) {}

BT::NodeStatus SelectBestTargetAction::tick()
{
  auto radar = getInput<rm_decision_interfaces::msg::RMUCEnemyTracks>("radar_tracks");
  if (!radar || radar->enemy_count == 0) {
    setOutput("out_target", std::string(""));
    return BT::NodeStatus::FAILURE;
  }

  double px = 0, py = 0, bx = 0, by = 0;
  getInput("pose_x", px);
  getInput("pose_y", py);
  getInput("base_x", bx);
  getInput("base_y", by);

  // 策略：优先选择距己方基地最近的敌人（威胁最大）
  // 若无基地附近目标（>5m），退回选距自身最近的
  int best_idx = -1;
  double best_base_dist = 1e9;
  int nearest_idx = -1;
  double nearest_self_dist = 1e9;

  for (size_t i = 0; i < radar->enemy_count && i < radar->enemy_x.size(); ++i) {
    double ex = radar->enemy_x[i];
    double ey = radar->enemy_y[i];
    double d_base = std::hypot(ex - bx, ey - by);
    double d_self = std::hypot(ex - px, ey - py);

    if (d_base < best_base_dist) {
      best_base_dist = d_base;
      best_idx = static_cast<int>(i);
    }
    if (d_self < nearest_self_dist) {
      nearest_self_dist = d_self;
      nearest_idx = static_cast<int>(i);
    }
  }

  int chosen = (best_base_dist < 5.0) ? best_idx : nearest_idx;
  if (chosen < 0) {
    setOutput("out_target", std::string(""));
    return BT::NodeStatus::FAILURE;
  }

  std::string target_str =
    std::to_string(radar->enemy_robot_id[chosen]) + ":" +
    std::to_string(radar->enemy_x[chosen]) + ":" +
    std::to_string(radar->enemy_y[chosen]);
  setOutput("out_target", target_str);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::SelectBestTargetAction>("SelectBestTarget");
}
