#include "rm_behavior_tree/plugins/rmuc_2026/action/objective_patrol.hpp"
#include "behaviortree_cpp/blackboard.h"
#include <algorithm>
#include <cmath>
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
      pts.push_back({x, y, 0, "patrol"});
    } catch (...) {
      // 忽略格式错误的巡逻点
    }
  }
  return pts;
}

BT::NodeStatus ObjectivePatrolAction::tick()
{
  double px = 0, py = 0, obj_x = 0, obj_y = 0;
  double fortress_x = 0, fortress_y = 0;
  getInput("pose_x", px);
  getInput("pose_y", py);
  getInput("objective_x", obj_x);
  getInput("objective_y", obj_y);
  getInput("fortress_area_x", fortress_x);
  getInput("fortress_area_y", fortress_y);

  std::string obj_name;
  getInput("objective_name", obj_name);

  bool patrol_enable = false;
  getInput("patrol_enable", patrol_enable);

  std::string wpts_str;
  getInput("patrol_waypoints", wpts_str);

  bool out_alive_patrol_enable = false;
  getInput("out_alive_patrol_enable", out_alive_patrol_enable);

  std::string out_alive_wpts_str;
  getInput("out_alive_patrol_waypoints", out_alive_wpts_str);

  auto * root_bb = config().blackboard->rootBlackboard();
  const std::string state_prefix = "objective_patrol." + name() + ".";

  auto ensure_int = [&](const std::string & key, int default_val) {
    try {
      (void)root_bb->get<int>(key);
    } catch (...) {
      root_bb->set<int>(key, default_val);
    }
  };
  auto ensure_bool = [&](const std::string & key, bool default_val) {
    try {
      (void)root_bb->get<bool>(key);
    } catch (...) {
      root_bb->set<bool>(key, default_val);
    }
  };
  auto ensure_int64 = [&](const std::string & key, int64_t default_val) {
    try {
      (void)root_bb->get<int64_t>(key);
    } catch (...) {
      root_bb->set<int64_t>(key, default_val);
    }
  };
  auto ensure_double = [&](const std::string & key, double default_val) {
    try {
      (void)root_bb->get<double>(key);
    } catch (...) {
      root_bb->set<double>(key, default_val);
    }
  };
  auto publish_goal = [&](double goal_x, double goal_y) {
    constexpr double kGoalChangeEps = 1e-3;
    const std::string last_x_key = state_prefix + "last_goal_x";
    const std::string last_y_key = state_prefix + "last_goal_y";
    const std::string have_last_key = state_prefix + "has_last_goal";
    ensure_double(last_x_key, goal_x);
    ensure_double(last_y_key, goal_y);
    ensure_bool(have_last_key, false);

    const bool have_last = root_bb->get<bool>(have_last_key);
    const double last_x = root_bb->get<double>(last_x_key);
    const double last_y = root_bb->get<double>(last_y_key);
    const bool changed = have_last && (
      std::fabs(goal_x - last_x) > kGoalChangeEps ||
      std::fabs(goal_y - last_y) > kGoalChangeEps);

    root_bb->set<double>(last_x_key, goal_x);
    root_bb->set<double>(last_y_key, goal_y);
    root_bb->set<bool>(have_last_key, true);
    setOutput("goal_x", goal_x);
    setOutput("goal_y", goal_y);
    setOutput("goal_changed", changed);
  };

  ensure_int(state_prefix + "current_idx", 0);
  ensure_bool(state_prefix + "was_arrived", false);
  ensure_int64(state_prefix + "arrived_time_ms", 0);

  const bool outpost_destroyed_objective = obj_name == "TRAPEZOIDAL_HIGHLAND";
  const bool outpost_alive_objective = obj_name == "CENTRAL_HIGHLAND";

  if (!outpost_destroyed_objective && !(outpost_alive_objective && out_alive_patrol_enable)) {
    // 直接输出战略目标坐标
    publish_goal(obj_x, obj_y);
    root_bb->set<int>(state_prefix + "current_idx", 0);
    root_bb->set<bool>(state_prefix + "was_arrived", false);
    root_bb->set<int64_t>(state_prefix + "arrived_time_ms", 0);
    return BT::NodeStatus::SUCCESS;
  }

  double arrive_radius = 0.5;
  int ladder_hold_ms = 5000;
  int fortress_hold_ms = 5000;
  int patrol_hold_ms = 5000;
  getInput("arrive_radius", arrive_radius);
  getInput("ladder_time", ladder_hold_ms);
  getInput("fortress_time", fortress_hold_ms);
  getInput("patrol_hold_ms", patrol_hold_ms);
  ladder_hold_ms = std::max(0, ladder_hold_ms);
  fortress_hold_ms = std::max(0, fortress_hold_ms);
  patrol_hold_ms = std::max(0, patrol_hold_ms);

  // 每 tick 重新构建巡逻环，状态保存在 root blackboard，避免节点实例重建导致状态丢失
  std::vector<Pt> cycle;
  if (outpost_alive_objective) {
    cycle.push_back({obj_x, obj_y, patrol_hold_ms, "central"});
    auto alive_patrol_pts = parseWaypoints(out_alive_wpts_str);
    for (auto & pt : alive_patrol_pts) {
      pt.hold_ms = patrol_hold_ms;
      pt.label = "out_alive_patrol";
    }
    cycle.insert(cycle.end(), alive_patrol_pts.begin(), alive_patrol_pts.end());
  } else {
    cycle.push_back({obj_x, obj_y, ladder_hold_ms, "ladder"});
    cycle.push_back({fortress_x, fortress_y, fortress_hold_ms, "fortress"});

    if (patrol_enable) {
      auto patrol_pts = parseWaypoints(wpts_str);
      for (auto & pt : patrol_pts) {
        pt.hold_ms = patrol_hold_ms;
      }
      cycle.insert(cycle.end(), patrol_pts.begin(), patrol_pts.end());
    }
  }

  if (cycle.empty()) {
    publish_goal(obj_x, obj_y);
    return BT::NodeStatus::SUCCESS;
  }

  int current_idx = root_bb->get<int>(state_prefix + "current_idx");
  bool was_arrived = root_bb->get<bool>(state_prefix + "was_arrived");
  int64_t arrived_time_ms = root_bb->get<int64_t>(state_prefix + "arrived_time_ms");

  current_idx = current_idx % static_cast<int>(cycle.size());

  auto & target = cycle[current_idx];
  hold_ms_cache_ = target.hold_ms;
  double dist = std::hypot(target.x - px, target.y - py);
  // 迟滞判定：已到达后用 2 倍半径防止自转漂移导致反复切换
  double effective_radius = was_arrived ? arrive_radius * 2.0 : arrive_radius;
  bool arrived = dist < effective_radius;

  auto now = std::chrono::steady_clock::now();
  const int64_t now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    now.time_since_epoch()).count();

  if (arrived && !was_arrived) {
    // 刚到达
    was_arrived = true;
    arrived_time_ms = now_ms;
  } else if (!arrived) {
    was_arrived = false;
    arrived_time_ms = 0;
  }

  if (was_arrived && arrived_time_ms > 0) {
    const int64_t hold_elapsed_ms = now_ms - arrived_time_ms;
    if (hold_elapsed_ms >= target.hold_ms) {
      // 停留时间到，切换下一个点
      int old_idx = current_idx;
      current_idx = (current_idx + 1) % static_cast<int>(cycle.size());
      was_arrived = false;
      arrived_time_ms = 0;
      fprintf(stderr,
              "[ObjectivePatrol] hold done -> idx %d->%d %s goal=(%.2f,%.2f)\n",
              old_idx, current_idx, cycle[current_idx].label,
              cycle[current_idx].x, cycle[current_idx].y);
    }
  }

  root_bb->set<int>(state_prefix + "current_idx", current_idx);
  root_bb->set<bool>(state_prefix + "was_arrived", was_arrived);
  root_bb->set<int64_t>(state_prefix + "arrived_time_ms", arrived_time_ms);

  // 每 5 秒打印一次巡逻状态，便于调试
  {
    static auto last_diag = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::seconds>(now - last_diag).count() >= 5) {
      last_diag = now;
      long hold_elapsed_ms = (was_arrived && arrived_time_ms > 0) ?
        static_cast<long>(now_ms - arrived_time_ms) : 0;
      fprintf(stderr,
              "[ObjectivePatrol] destroyed_enable=%d alive_enable=%d obj=%s wpts=%zuB alive_wpts=%zuB active=1 idx=%d/%zu target=%s arrived=%d hold=%ldms/%dms dist=%.2f\n",
              patrol_enable, out_alive_patrol_enable, obj_name.c_str(),
              wpts_str.size(), out_alive_wpts_str.size(),
              current_idx, cycle.size(), target.label, was_arrived,
              hold_elapsed_ms, hold_ms_cache_, dist);
    }
  }

  publish_goal(cycle[current_idx].x, cycle[current_idx].y);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::ObjectivePatrolAction>("ObjectivePatrol");
}
