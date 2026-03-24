#include "rm_behavior_tree/plugins/rmul_2026/action/snap_goal_to_free_space.hpp"

#include <cmath>

#include "behaviortree_ros2/plugins.hpp"

namespace rm_behavior_tree
{

SnapGoalToFreeSpaceAction::SnapGoalToFreeSpaceAction(
  const std::string & name,
  const BT::NodeConfig & conf,
  const BT::RosNodeParams & params)
: BT::SyncActionNode(name, conf)
{
  node_ = params.nh;
}

void SnapGoalToFreeSpaceAction::costmapCallback(
  const nav2_msgs::msg::Costmap::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(costmap_mutex_);
  latest_costmap_ = msg;
}

uint8_t SnapGoalToFreeSpaceAction::getCost(double world_x, double world_y) const
{
  std::lock_guard<std::mutex> lock(costmap_mutex_);
  if (!latest_costmap_ || latest_costmap_->data.empty()) {
    return 255;  // no map available → treat as lethal
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
    return 255;  // out of bounds
  }

  const size_t idx = static_cast<size_t>(gy) * meta.size_x + static_cast<size_t>(gx);
  if (idx >= latest_costmap_->data.size()) {
    return 255;
  }
  return latest_costmap_->data[idx];
}

bool SnapGoalToFreeSpaceAction::spiralSearch(
  double cx, double cy, double max_radius, double step,
  uint8_t threshold, double & out_x, double & out_y) const
{
  // 方向向量：8 方向（0°, 45°, 90°, ... 315°）
  static const double dirs[][2] = {
    { 1.0,  0.0}, { 0.707,  0.707}, { 0.0,  1.0}, {-0.707,  0.707},
    {-1.0,  0.0}, {-0.707, -0.707}, { 0.0, -1.0}, { 0.707, -0.707}
  };

  for (double r = step; r <= max_radius; r += step) {
    for (const auto & d : dirs) {
      const double tx = cx + d[0] * r;
      const double ty = cy + d[1] * r;
      if (getCost(tx, ty) < threshold) {
        out_x = tx;
        out_y = ty;
        return true;
      }
    }
  }
  return false;  // 搜索范围内无可达点
}

BT::NodeStatus SnapGoalToFreeSpaceAction::tick()
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

  // spin 一次以获取最新 costmap
  if (node_) {
    rclcpp::spin_some(node_);
  }

  // 2. 读取目标点
  double gx = 0.0, gy = 0.0;
  bool has_goal = false;

  auto res_pose = getInput<geometry_msgs::msg::PoseStamped>("goal_pose");
  if (res_pose) {
    gx = res_pose->pose.position.x;
    gy = res_pose->pose.position.y;
    has_goal = true;
  } else {
    auto rx = getInput<double>("goal_x");
    auto ry = getInput<double>("goal_y");
    if (rx && ry) {
      gx = rx.value();
      gy = ry.value();
      has_goal = true;
    }
  }

  if (!has_goal) {
    return BT::NodeStatus::FAILURE;
  }

  // 3. 检查目标点代价
  uint8_t lethal_threshold = 253;
  getInput("lethal_threshold", lethal_threshold);

  const uint8_t cost = getCost(gx, gy);
  if (cost < lethal_threshold) {
    // 目标点可达，原样输出
    setOutput("goal_x", gx);
    setOutput("goal_y", gy);
    if (res_pose) {
      setOutput("goal_pose", res_pose.value());
    }
    return BT::NodeStatus::SUCCESS;
  }

  // 4. 目标点在障碍内，螺旋搜索最近可达点
  double max_radius = 1.0;
  double search_step = 0.1;
  getInput("max_search_radius", max_radius);
  getInput("search_step", search_step);

  double adj_x = 0.0, adj_y = 0.0;
  if (!spiralSearch(gx, gy, max_radius, search_step, lethal_threshold, adj_x, adj_y)) {
    if (node_) {
      RCLCPP_WARN(node_->get_logger(),
        "[SnapGoal] Goal (%.2f, %.2f) cost=%d in obstacle, no free cell within %.1fm",
        gx, gy, cost, max_radius);
    }
    return BT::NodeStatus::FAILURE;
  }

  if (node_) {
    RCLCPP_INFO(node_->get_logger(),
      "[SnapGoal] Adjusted (%.2f, %.2f) → (%.2f, %.2f) (cost %d→%d)",
      gx, gy, adj_x, adj_y, cost, getCost(adj_x, adj_y));
  }

  // 5. 输出修正后的目标
  setOutput("goal_x", adj_x);
  setOutput("goal_y", adj_y);

  geometry_msgs::msg::PoseStamped adjusted_pose;
  if (res_pose) {
    adjusted_pose = res_pose.value();
  } else {
    adjusted_pose.header.frame_id = "map";
    adjusted_pose.pose.orientation.w = 1.0;
  }
  adjusted_pose.pose.position.x = adj_x;
  adjusted_pose.pose.position.y = adj_y;
  setOutput("goal_pose", adjusted_pose);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::SnapGoalToFreeSpaceAction, "SnapGoalToFreeSpace");
