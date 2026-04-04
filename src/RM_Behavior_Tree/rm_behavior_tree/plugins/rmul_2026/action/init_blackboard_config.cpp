#include "rm_behavior_tree/plugins/rmul_2026/action/init_blackboard_config.hpp"

#include <algorithm>

namespace rm_behavior_tree
{

InitBlackboardConfigAction::InitBlackboardConfigAction(
  const std::string& name,
  const BT::NodeConfig& conf,
  const BT::RosNodeParams& params)
: BT::SyncActionNode(name, conf)
{
  node_ = params.nh;
}

BT::NodeStatus InitBlackboardConfigAction::tick()
{
  if (initialized_) {
    return BT::NodeStatus::SUCCESS;
  }

  std::uint64_t heal_wait_ms        = HEAL_WAIT_MS_DEFAULT;
  int           heal_min_hp         = HEAL_MIN_HP_DEFAULT;
  int           hp_high_threshold    = HP_HIGH_THRESHOLD_DEFAULT;
  int           hp_medium_threshold  = HP_MEDIUM_THRESHOLD_DEFAULT;
  std::uint64_t search_timeout_ms   = SEARCH_TIMEOUT_MS_DEFAULT;
  std::uint64_t recovery_timeout_ms = RECOVERY_TIMEOUT_MS_DEFAULT;
  double        arrive_radius       = ARRIVE_RADIUS_DEFAULT;
  double        stuck_check_radius  = STUCK_CHECK_RADIUS_DEFAULT;

  double supply_goal_x = RED_SUPPLY_GOAL_X_DEFAULT;
  double supply_goal_y = RED_SUPPLY_GOAL_Y_DEFAULT;
  double control_zone_goal_x = RED_CONTROL_ZONE_X_DEFAULT;
  double control_zone_goal_y = RED_CONTROL_ZONE_Y_DEFAULT;
  double control_zone_offset_x = CONTROL_ZONE_OFFSET_X_DEFAULT;
  double control_zone_offset_y = CONTROL_ZONE_OFFSET_Y_DEFAULT;

  // 直接从 ROS 参数获取红蓝方阵营，避免依赖 namespace（实车不带 ns）
  bool is_blue = false;

  if (node_ != nullptr) {
    // 声明并获取参数
    if (!node_->has_parameter("is_blue_team")) {
      node_->declare_parameter("is_blue_team", false);
    }
    if (!node_->has_parameter("supply_heal_min_hp")) {
      node_->declare_parameter("supply_heal_min_hp", HEAL_MIN_HP_DEFAULT);
    }
    if (!node_->has_parameter("hp_high_threshold")) {
      node_->declare_parameter("hp_high_threshold", HP_HIGH_THRESHOLD_DEFAULT);
    }
    if (!node_->has_parameter("hp_medium_threshold")) {
      node_->declare_parameter("hp_medium_threshold", HP_MEDIUM_THRESHOLD_DEFAULT);
    }
    node_->get_parameter("supply_heal_min_hp", heal_min_hp);
    node_->get_parameter("hp_high_threshold", hp_high_threshold);
    node_->get_parameter("hp_medium_threshold", hp_medium_threshold);

    if (!node_->has_parameter("red_supply_goal_x")) node_->declare_parameter("red_supply_goal_x", RED_SUPPLY_GOAL_X_DEFAULT);
    if (!node_->has_parameter("red_supply_goal_y")) node_->declare_parameter("red_supply_goal_y", RED_SUPPLY_GOAL_Y_DEFAULT);
    if (!node_->has_parameter("red_control_zone_x")) node_->declare_parameter("red_control_zone_x", RED_CONTROL_ZONE_X_DEFAULT);
    if (!node_->has_parameter("red_control_zone_y")) node_->declare_parameter("red_control_zone_y", RED_CONTROL_ZONE_Y_DEFAULT);

    if (!node_->has_parameter("blue_supply_goal_x")) node_->declare_parameter("blue_supply_goal_x", BLUE_SUPPLY_GOAL_X_DEFAULT);
    if (!node_->has_parameter("blue_supply_goal_y")) node_->declare_parameter("blue_supply_goal_y", BLUE_SUPPLY_GOAL_Y_DEFAULT);
    if (!node_->has_parameter("blue_control_zone_x")) node_->declare_parameter("blue_control_zone_x", BLUE_CONTROL_ZONE_X_DEFAULT);
    if (!node_->has_parameter("blue_control_zone_y")) node_->declare_parameter("blue_control_zone_y", BLUE_CONTROL_ZONE_Y_DEFAULT);

    if (!node_->has_parameter("control_zone_offset_x")) node_->declare_parameter("control_zone_offset_x", CONTROL_ZONE_OFFSET_X_DEFAULT);
    if (!node_->has_parameter("control_zone_offset_y")) node_->declare_parameter("control_zone_offset_y", CONTROL_ZONE_OFFSET_Y_DEFAULT);

    node_->get_parameter("is_blue_team", is_blue);

    if (is_blue) {
      node_->get_parameter("blue_supply_goal_x", supply_goal_x);
      node_->get_parameter("blue_supply_goal_y", supply_goal_y);
      node_->get_parameter("blue_control_zone_x", control_zone_goal_x);
      node_->get_parameter("blue_control_zone_y", control_zone_goal_y);
      RCLCPP_INFO(node_->get_logger(), "Team parsing result: BLUE. Applied BLUE fallbacks. Supply(%.2f, %.2f) Control(%.2f, %.2f)", 
                  supply_goal_x, supply_goal_y, control_zone_goal_x, control_zone_goal_y);
    } else {
      node_->get_parameter("red_supply_goal_x", supply_goal_x);
      node_->get_parameter("red_supply_goal_y", supply_goal_y);
      node_->get_parameter("red_control_zone_x", control_zone_goal_x);
      node_->get_parameter("red_control_zone_y", control_zone_goal_y);
      RCLCPP_INFO(node_->get_logger(), "Team parsing result: RED. Applied RED fallbacks. Supply(%.2f, %.2f) Control(%.2f, %.2f)", 
                  supply_goal_x, supply_goal_y, control_zone_goal_x, control_zone_goal_y);
    }

    node_->get_parameter("control_zone_offset_x", control_zone_offset_x);
    node_->get_parameter("control_zone_offset_y", control_zone_offset_y);

    if (!node_->has_parameter("arrive_radius")) node_->declare_parameter("arrive_radius", ARRIVE_RADIUS_DEFAULT);
    node_->get_parameter("arrive_radius", arrive_radius);

    if (!node_->has_parameter("stuck_check_radius")) node_->declare_parameter("stuck_check_radius", STUCK_CHECK_RADIUS_DEFAULT);
    node_->get_parameter("stuck_check_radius", stuck_check_radius);
  }

  // 安全夹紧
  heal_min_hp = std::clamp(heal_min_hp, 0, 400);
  hp_high_threshold = std::clamp(hp_high_threshold, 1, 400);
  hp_medium_threshold = std::clamp(hp_medium_threshold, 0, hp_high_threshold - 1);
  if (arrive_radius < 0.05) {
    arrive_radius = 0.05;
  }

  // 写入黑板 cfg.*
  setOutput("heal_wait_ms", heal_wait_ms);
  setOutput("heal_min_hp", heal_min_hp);
  setOutput("hp_high_threshold", hp_high_threshold);
  setOutput("hp_medium_threshold", hp_medium_threshold);
  setOutput("search_timeout_ms", search_timeout_ms);
  setOutput("recovery_timeout_ms", recovery_timeout_ms);
  setOutput("supply_goal_x", supply_goal_x);
  setOutput("supply_goal_y", supply_goal_y);
  setOutput("control_zone_goal_x", control_zone_goal_x);
  setOutput("control_zone_goal_y", control_zone_goal_y);
  setOutput("control_zone_offset_x", control_zone_offset_x);
  setOutput("control_zone_offset_y", control_zone_offset_y);
  setOutput("arrive_radius", arrive_radius);
  setOutput("stuck_check_radius", stuck_check_radius);

  initialized_ = true;
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::InitBlackboardConfigAction, "InitBlackboardConfig");
