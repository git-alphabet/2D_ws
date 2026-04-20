#include "rm_behavior_tree/plugins/rmuc_2026/action/objective_patrol.hpp"
#include <sstream>

namespace rm_behavior_tree
{

ObjectivePatrolAction::ObjectivePatrolAction(
  const std::string & name, const BT::NodeConfig & conf)
: BT::SyncActionNode(name, conf) {}

std::vector<ObjectivePatrolAction::Pt>
ObjectivePatrolAction::parseWaypoints(const std::string & s)
{
  std::vector<Pt> pts;
  if (s.empty()) return pts;

  std::istringstream ss(s);
  std::string token;
  while (std::getline(ss, token, ';')) {
    auto comma = token.find(',');
    if (comma == std::string::npos) continue;
    try {
      double x = std::stod(token.substr(0, comma));
      double y = std::stod(token.substr(comma + 1));
      pts.push_back({x, y});
    } catch (...) {
      // 忽略格式错误的巡逻点
    }
  }
  return pts;
}

BT::NodeStatus ObjectivePatrolAction::tick()
{
  double px = 0, py = 0, obj_x = 0, obj_y = 0;
  getInput("pose_x", px);
  getInput("pose_y", py);
  getInput("objective_x", obj_x);
  getInput("objective_y", obj_y);

  std::string obj_name;
  getInput("objective_name", obj_name);

  bool patrol_enable = false;
  getInput("patrol_enable", patrol_enable);

  std::string wpts_str;
  getInput("patrol_waypoints", wpts_str);

  // 巡逻仅在 TRAPEZOIDAL_HIGHLAND + 启用 + 有巡逻点 时激活
  bool patrol_active = patrol_enable
                       && obj_name == "TRAPEZOIDAL_HIGHLAND"
                       && !wpts_str.empty();

  // 每 5 秒打印一次巡逻状态，便于调试
  {
    static auto last_diag = std::chrono::steady_clock::now();
    auto now_diag = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::seconds>(now_diag - last_diag).count() >= 5) {
      last_diag = now_diag;
      long hold_elapsed_ms = 0;
      if (was_arrived_) {
        hold_elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
          now_diag - arrived_time_).count();
      }
      fprintf(stderr, "[ObjectivePatrol] enable=%d obj=%s wpts=%zuB active=%d idx=%d/%zu arrived=%d hold=%ldms/%dms dist=%.2f\n",
              patrol_enable, obj_name.c_str(), wpts_str.size(), patrol_active,
              current_idx_, cycle_.size(), was_arrived_, hold_elapsed_ms, hold_ms_cache_,
              cycle_.empty() ? -1.0 : std::hypot(cycle_[current_idx_ % cycle_.size()].x - px,
                                                  cycle_[current_idx_ % cycle_.size()].y - py));
    }
  }

  if (!patrol_active) {
    // 直接输出战略目标坐标
    setOutput("goal_x", obj_x);
    setOutput("goal_y", obj_y);
    // 重置巡逻状态
    current_idx_ = 0;
    was_arrived_ = false;
    cycle_.clear();
    return BT::NodeStatus::SUCCESS;
  }

  // 检测配置是否变化，重建巡逻循环
  if (wpts_str != last_waypoints_str_ || obj_x != last_obj_x_ || obj_y != last_obj_y_) {
    last_waypoints_str_ = wpts_str;
    last_obj_x_ = obj_x;
    last_obj_y_ = obj_y;

    cycle_.clear();
    cycle_.push_back({obj_x, obj_y});  // 战略目标作为第一个点
    auto patrol_pts = parseWaypoints(wpts_str);
    cycle_.insert(cycle_.end(), patrol_pts.begin(), patrol_pts.end());

    current_idx_ = 0;
    was_arrived_ = false;
  }

  if (cycle_.empty()) {
    setOutput("goal_x", obj_x);
    setOutput("goal_y", obj_y);
    return BT::NodeStatus::SUCCESS;
  }

  // 确保索引有效
  current_idx_ = current_idx_ % static_cast<int>(cycle_.size());

  double arrive_radius = 0.5;
  int hold_ms = 5000;
  getInput("arrive_radius", arrive_radius);
  getInput("patrol_hold_ms", hold_ms);
  hold_ms_cache_ = hold_ms;

  auto & target = cycle_[current_idx_];
  double dist = std::hypot(target.x - px, target.y - py);
  // 迟滞判定：已到达后用 2 倍半径防止自转漂移导致反复切换
  double effective_radius = was_arrived_ ? arrive_radius * 2.0 : arrive_radius;
  bool arrived = dist < effective_radius;

  if (arrived && !was_arrived_) {
    // 刚到达
    was_arrived_ = true;
    arrived_time_ = std::chrono::steady_clock::now();
  } else if (!arrived) {
    was_arrived_ = false;
  }

  if (was_arrived_) {
    auto elapsed = std::chrono::steady_clock::now() - arrived_time_;
    if (std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count() >= hold_ms) {
      // 停留时间到，切换下一个点
      int old_idx = current_idx_;
      current_idx_ = (current_idx_ + 1) % static_cast<int>(cycle_.size());
      was_arrived_ = false;
      fprintf(stderr, "[ObjectivePatrol] hold done → idx %d→%d goal=(%.2f,%.2f)\n",
              old_idx, current_idx_, cycle_[current_idx_].x, cycle_[current_idx_].y);
    }
  }

  setOutput("goal_x", cycle_[current_idx_].x);
  setOutput("goal_y", cycle_[current_idx_].y);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::ObjectivePatrolAction>("ObjectivePatrol");
}
