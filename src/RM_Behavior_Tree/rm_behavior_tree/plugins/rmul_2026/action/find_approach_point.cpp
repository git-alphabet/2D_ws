#include "rm_behavior_tree/plugins/rmul_2026/action/find_approach_point.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "behaviortree_ros2/plugins.hpp"

namespace rm_behavior_tree
{

FindApproachPointAction::FindApproachPointAction(
  const std::string & name,
  const BT::NodeConfig & conf,
  const BT::RosNodeParams & params)
: BT::SyncActionNode(name, conf)
{
  node_ = params.nh;
}

void FindApproachPointAction::costmapCallback(
  const nav2_msgs::msg::Costmap::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(costmap_mutex_);
  latest_costmap_ = msg;
}

uint8_t FindApproachPointAction::getCost(double world_x, double world_y) const
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

bool FindApproachPointAction::isPathClear(
  double ax, double ay, double bx, double by,
  uint8_t lethal_threshold, double sample_step) const
{
  const double dx = bx - ax;
  const double dy = by - ay;
  const double dist = std::sqrt(dx * dx + dy * dy);
  if (dist < sample_step) {
    return true;
  }
  const int num_samples = static_cast<int>(std::ceil(dist / sample_step));
  for (int i = 1; i < num_samples; ++i) {
    const double t = static_cast<double>(i) / num_samples;
    const double sx = ax + dx * t;
    const double sy = ay + dy * t;
    if (getCost(sx, sy) >= lethal_threshold) {
      return false;
    }
  }
  return true;
}

BT::NodeStatus FindApproachPointAction::tick()
{
  const auto now = std::chrono::steady_clock::now();

  // 心跳检测：距上次 tick 超过 2 秒，说明树已切走，重置状态
  if (last_tick_time_set_) {
    const auto gap_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      now - last_tick_time_).count();
    if (gap_ms > 2000) {
      has_committed_ = false;
    }
  }
  last_tick_time_ = now;
  last_tick_time_set_ = true;

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
  auto rx = getInput<double>("robot_x");
  auto ry = getInput<double>("robot_y");
  auto gx = getInput<double>("goal_x");
  auto gy = getInput<double>("goal_y");

  if (!rx || !ry || !gx || !gy) {
    RCLCPP_WARN(node_->get_logger(), "[FindApproachPoint] Missing required inputs");
    return BT::NodeStatus::FAILURE;
  }

  const double robot_x = rx.value();
  const double robot_y = ry.value();
  const double goal_x = gx.value();
  const double goal_y = gy.value();

  double search_radius = 4.5;
  double min_radius = 0.3;
  uint8_t cost_threshold = 150;
  getInput("search_radius", search_radius);
  getInput("min_radius", min_radius);
  getInput("cost_threshold", cost_threshold);

  // 3. 检查已提交的接近点
  if (has_committed_) {
    const uint8_t ccost = getCost(committed_x_, committed_y_);
    const double cdist = std::hypot(robot_x - committed_x_, robot_y - committed_y_);

    if (ccost < 235 && cdist < MAX_COMMIT_DISTANCE) {
      outputResult(committed_x_, committed_y_);
      return BT::NodeStatus::SUCCESS;
    }
    // 已提交点失效
    has_committed_ = false;
    if (node_) {
      RCLCPP_INFO(node_->get_logger(),
        "[FindApproachPoint] Committed point (%.2f,%.2f) invalidated (cost=%d, dist=%.2f)",
        committed_x_, committed_y_, ccost, cdist);
    }
  }

  // 4. 以目标点为圆心，同心环搜索代价最低的可达点
  static const double dirs[][2] = {
    { 1.0,  0.0}, { 0.866,  0.5}, { 0.5,  0.866}, { 0.0,  1.0},
    {-0.5,  0.866}, {-0.866,  0.5}, {-1.0,  0.0}, {-0.866, -0.5},
    {-0.5, -0.866}, { 0.0, -1.0}, { 0.5, -0.866}, { 0.866, -0.5},
    { 0.707,  0.707}, {-0.707,  0.707}, {-0.707, -0.707}, { 0.707, -0.707}
  };

  struct Candidate {
    double x, y;
    uint8_t cost;
    double dist_to_goal;
  };

  std::vector<Candidate> candidates;
  const double step = 0.15;

  for (double r = min_radius; r <= search_radius; r += step) {
    for (const auto & d : dirs) {
      const double px = goal_x + d[0] * r;
      const double py = goal_y + d[1] * r;
      const uint8_t cost = getCost(px, py);

      if (cost >= cost_threshold) {
        continue;
      }

      // 检查从机器人到候选点路径是否无致命障碍
      if (!isPathClear(robot_x, robot_y, px, py)) {
        continue;
      }

      const double dist_to_goal = std::hypot(px - goal_x, py - goal_y);
      candidates.push_back({px, py, cost, dist_to_goal});
    }
  }

  if (candidates.empty()) {
    if (node_) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
        "[FindApproachPoint] No approach point found near goal (%.2f,%.2f) within %.1fm",
        goal_x, goal_y, search_radius);
    }
    return BT::NodeStatus::FAILURE;
  }

  // 排序: 先按 cost 升序，同 cost 按距目标近优先
  std::sort(candidates.begin(), candidates.end(),
    [](const Candidate & a, const Candidate & b) {
      if (a.cost != b.cost) {
        return a.cost < b.cost;
      }
      return a.dist_to_goal < b.dist_to_goal;
    });

  has_committed_ = true;
  committed_x_ = candidates.front().x;
  committed_y_ = candidates.front().y;

  if (node_) {
    RCLCPP_INFO(node_->get_logger(),
      "[FindApproachPoint] goal(%.2f,%.2f) → approach(%.2f,%.2f) cost=%d [committed]",
      goal_x, goal_y, committed_x_, committed_y_, candidates.front().cost);
  }

  outputResult(committed_x_, committed_y_);
  return BT::NodeStatus::SUCCESS;
}

void FindApproachPointAction::outputResult(double x, double y)
{
  setOutput("approach_x", x);
  setOutput("approach_y", y);

  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = 0.0;
  pose.pose.orientation.w = 1.0;
  setOutput("approach_pose", pose);
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::FindApproachPointAction, "FindApproachPoint");
