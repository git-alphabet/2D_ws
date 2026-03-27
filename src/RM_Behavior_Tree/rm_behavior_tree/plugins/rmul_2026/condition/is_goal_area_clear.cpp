#include "rm_behavior_tree/plugins/rmul_2026/condition/is_goal_area_clear.hpp"

#include <cmath>

#include "behaviortree_ros2/plugins.hpp"

namespace rm_behavior_tree
{

IsGoalAreaClearCondition::IsGoalAreaClearCondition(
  const std::string & name,
  const BT::NodeConfig & conf,
  const BT::RosNodeParams & params)
: BT::ConditionNode(name, conf)
{
  node_ = params.nh;
}

void IsGoalAreaClearCondition::costmapCallback(
  const nav2_msgs::msg::Costmap::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(costmap_mutex_);
  latest_costmap_ = msg;
}

uint8_t IsGoalAreaClearCondition::getCost(double world_x, double world_y) const
{
  std::lock_guard<std::mutex> lock(costmap_mutex_);
  if (!latest_costmap_ || latest_costmap_->data.empty()) {
    return 255;
  }

  const auto & meta = latest_costmap_->metadata;
  const double res = meta.resolution;
  if (res <= 0.0) {
    return 255;
  }

  const int gx = static_cast<int>(std::floor((world_x - meta.origin.position.x) / res));
  const int gy = static_cast<int>(std::floor((world_y - meta.origin.position.y) / res));

  if (gx < 0 || gy < 0 ||
      gx >= static_cast<int>(meta.size_x) ||
      gy >= static_cast<int>(meta.size_y))
  {
    return 255;
  }

  const size_t idx = static_cast<size_t>(gy) * meta.size_x + static_cast<size_t>(gx);
  if (idx >= latest_costmap_->data.size()) {
    return 255;
  }
  return latest_costmap_->data[idx];
}

BT::NodeStatus IsGoalAreaClearCondition::tick()
{
  // 1. 确保 costmap 订阅就绪
  std::string topic = "global_costmap/costmap_raw";
  getInput("costmap_topic", topic);

  if (!costmap_sub_ || subscribed_topic_ != topic) {
    if (node_) {
      costmap_sub_ = node_->create_subscription<nav2_msgs::msg::Costmap>(
        topic, rclcpp::QoS(1).best_effort(),
        [this](const nav2_msgs::msg::Costmap::SharedPtr msg) {
          costmapCallback(msg);
        });
      subscribed_topic_ = topic;
    }
  }

  if (node_) {
    rclcpp::spin_some(node_);
  }

  // 2. 读取输入
  auto gx_opt = getInput<double>("goal_x");
  auto gy_opt = getInput<double>("goal_y");
  if (!gx_opt || !gy_opt) {
    return BT::NodeStatus::FAILURE;
  }

  const double gx = gx_opt.value();
  const double gy = gy_opt.value();

  double check_radius = 0.3;
  int min_free_neighbors = 2;
  uint8_t free_threshold = 200;
  getInput("check_radius", check_radius);
  getInput("min_free_neighbors", min_free_neighbors);
  getInput("free_threshold", free_threshold);

  // 3. 检查目标点本身
  const uint8_t goal_cost = getCost(gx, gy);
  if (goal_cost < 253) {
    // 目标点不在致命障碍上 → 可以前往
    return BT::NodeStatus::SUCCESS;
  }

  // 4. 目标点在障碍上，检查8邻居
  static const double dirs[][2] = {
    {1, 0}, {0.707, 0.707}, {0, 1}, {-0.707, 0.707},
    {-1, 0}, {-0.707, -0.707}, {0, -1}, {0.707, -0.707}
  };

  int free_count = 0;
  for (const auto & d : dirs) {
    const uint8_t c = getCost(gx + d[0] * check_radius, gy + d[1] * check_radius);
    if (c < free_threshold) {
      ++free_count;
    }
  }

  if (free_count >= min_free_neighbors) {
    // 有足够多的邻居是自由的 → 可以前往（SnapGoalToFreeSpace会修正目标点）
    return BT::NodeStatus::SUCCESS;
  }

  // 目标区域被封死
  return BT::NodeStatus::FAILURE;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::IsGoalAreaClearCondition, "IsGoalAreaClear");
