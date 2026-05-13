#include "rm_behavior_tree/plugins/rmuc_2026/action/select_objective.hpp"
#include <chrono>
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
  getInput("cap_sustain_time", cap_sustain_time);
  getInput("stage_elapsed_time", stage_elapsed_time);
  getInput("outpost_alive", outpost_alive);

  std::string objective;
  double gx = 0, gy = 0;

  auto game_status = getInput<sp_msgs::msg::RMUCGameStatus>("game_status");
  const bool match_started = game_status && game_status->game_progress == 4;
  const auto now = std::chrono::steady_clock::now();

  if (cap_sustain_time <= 0) {
    cap_timer_done_ = true;
  } else if (match_started && !cap_timer_started_ && !cap_timer_done_) {
    cap_timer_started_ = true;
    cap_timer_start_ = now;
    fprintf(stderr,
      "[SelectObjective] cap_outpost timer started, cap_sustain_time=%d sec\n",
      cap_sustain_time);
  }

  if (cap_timer_started_ && !cap_timer_done_) {
    const int64_t elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      now - cap_timer_start_).count();
    if (elapsed_ms >= static_cast<int64_t>(cap_sustain_time) * 1000) {
      cap_timer_done_ = true;
      fprintf(stderr,
        "[SelectObjective] cap_outpost timer done, branch will stay completed\n");
    }
  }

  const bool in_cap_window = cap_timer_started_ && !cap_timer_done_;

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
      "[SelectObjective] game_elapsed_s=%d, cap_timer_started=%d, cap_timer_done=%d, outpost_alive=%d -> %s (%.2f, %.2f)\n",
      stage_elapsed_time, cap_timer_started_, cap_timer_done_, outpost_alive,
      objective.c_str(), gx, gy);
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
