#include "rm_behavior_tree/plugins/rmul_2026/action/init_blackboard_config.hpp"

#include <algorithm>

namespace rm_behavior_tree
{

InitBlackboardConfigAction::InitBlackboardConfigAction(
  const std::string& name,
  const BT::NodeConfig& conf,
  const BT::RosNodeParams& /*params*/)
: BT::SyncActionNode(name, conf)
{
}

BT::NodeStatus InitBlackboardConfigAction::tick()
{
  if (initialized_) {
    return BT::NodeStatus::SUCCESS;
  }

  std::uint64_t heal_wait_ms        = HEAL_WAIT_MS_DEFAULT;
  double        heal_min_ratio      = HEAL_MIN_RATIO_DEFAULT;
  std::uint64_t search_timeout_ms   = SEARCH_TIMEOUT_MS_DEFAULT;
  std::uint64_t recovery_timeout_ms = RECOVERY_TIMEOUT_MS_DEFAULT;
  double        supply_goal_x       = SUPPLY_GOAL_X_DEFAULT;
  double        supply_goal_y       = SUPPLY_GOAL_Y_DEFAULT;
  double        arrive_radius       = ARRIVE_RADIUS_DEFAULT;

  // 安全夹紧
  heal_min_ratio = std::clamp(heal_min_ratio, 0.0, 1.0);
  if (arrive_radius < 0.05) {
    arrive_radius = 0.05;
  }

  // 写入黑板 cfg.*
  setOutput("heal_wait_ms", heal_wait_ms);
  setOutput("heal_min_ratio", heal_min_ratio);
  setOutput("search_timeout_ms", search_timeout_ms);
  setOutput("recovery_timeout_ms", recovery_timeout_ms);
  setOutput("supply_goal_x", supply_goal_x);
  setOutput("supply_goal_y", supply_goal_y);
  setOutput("arrive_radius", arrive_radius);

  initialized_ = true;
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::InitBlackboardConfigAction, "InitBlackboardConfig");
