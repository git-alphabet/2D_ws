#include "rm_behavior_tree/plugins/rmul_2026/action/find_escape_point.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "behaviortree_ros2/plugins.hpp"

namespace rm_behavior_tree
{

FindEscapePointAction::FindEscapePointAction(
  const std::string & name,
  const BT::NodeConfig & conf,
  const BT::RosNodeParams & params)
: BT::SyncActionNode(name, conf)
{
  node_ = params.nh;
}

void FindEscapePointAction::costmapCallback(
  const nav2_msgs::msg::Costmap::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(costmap_mutex_);
  latest_costmap_ = msg;
}

uint8_t FindEscapePointAction::getCost(double world_x, double world_y) const
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

double FindEscapePointAction::getNeighborClearance(
  double wx, double wy, double step) const
{
  // 检查8个邻居的代价，返回平均空闲度 (253 - avg_cost)
  static const double dirs[][2] = {
    {1, 0}, {0.707, 0.707}, {0, 1}, {-0.707, 0.707},
    {-1, 0}, {-0.707, -0.707}, {0, -1}, {0.707, -0.707}
  };

  double total_cost = 0.0;
  int count = 0;
  for (const auto & d : dirs) {
    const uint8_t c = getCost(wx + d[0] * step, wy + d[1] * step);
    if (c < 255) {  // 只计入有效栅格
      total_cost += c;
      ++count;
    }
  }

  if (count == 0) {
    return 0.0;
  }
  return 253.0 - (total_cost / count);
}

bool FindEscapePointAction::searchPhase(
  double robot_x, double robot_y,
  double goal_x, double goal_y,
  double retreat_dx, double retreat_dy,
  const SearchParams & params,
  double & out_x, double & out_y) const
{
  // 8方向搜索
  static const double dirs[][2] = {
    { 1.0,  0.0}, { 0.707,  0.707}, { 0.0,  1.0}, {-0.707,  0.707},
    {-1.0,  0.0}, {-0.707, -0.707}, { 0.0, -1.0}, { 0.707, -0.707}
  };

  std::vector<CandidatePoint> candidates;

  for (double r = params.min_radius; r <= params.max_radius; r += params.step) {
    for (const auto & d : dirs) {
      const double px = robot_x + d[0] * r;
      const double py = robot_y + d[1] * r;
      const uint8_t cost = getCost(px, py);

      if (cost >= params.cost_threshold) {
        continue;
      }

      const double dx = px - goal_x;
      const double dy = py - goal_y;
      const double dist_goal = std::sqrt(dx * dx + dy * dy);

      // 方向偏好：候选点相对机器人的向量 · 后退方向
      const double vx = px - robot_x;
      const double vy = py - robot_y;
      const double dot = vx * retreat_dx + vy * retreat_dy;

      const double clearance = getNeighborClearance(px, py, params.step);

      candidates.push_back({px, py, dist_goal, dot, clearance});
    }
  }

  if (candidates.empty()) {
    return false;
  }

  if (params.prefer_retreat) {
    // 优先后退方向的点(dot>0)，然后按 距离目标近 + clearance bonus 排序
    // score 越小越好: dist_to_goal - 0.1 * clearance, 后退方向优先
    std::sort(candidates.begin(), candidates.end(),
      [](const CandidatePoint & a, const CandidatePoint & b) {
        const bool a_retreat = a.retreat_dot > 0.0;
        const bool b_retreat = b.retreat_dot > 0.0;
        if (a_retreat != b_retreat) {
          return a_retreat;  // 后退方向优先
        }
        const double score_a = a.dist_to_goal - 0.1 * a.clearance;
        const double score_b = b.dist_to_goal - 0.1 * b.clearance;
        return score_a < score_b;
      });
  } else {
    // 不限方向：按 距离目标近 + clearance bonus 排序
    std::sort(candidates.begin(), candidates.end(),
      [](const CandidatePoint & a, const CandidatePoint & b) {
        const double score_a = a.dist_to_goal - 0.1 * a.clearance;
        const double score_b = b.dist_to_goal - 0.1 * b.clearance;
        return score_a < score_b;
      });
  }

  out_x = candidates.front().x;
  out_y = candidates.front().y;
  return true;
}

BT::NodeStatus FindEscapePointAction::tick()
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
  auto rx = getInput<double>("robot_x");
  auto ry = getInput<double>("robot_y");
  auto gx = getInput<double>("goal_x");
  auto gy = getInput<double>("goal_y");

  if (!rx || !ry || !gx || !gy) {
    RCLCPP_WARN(node_->get_logger(), "[FindEscapePoint] Missing input ports");
    return BT::NodeStatus::FAILURE;
  }

  const double robot_x = rx.value();
  const double robot_y = ry.value();
  const double goal_x = gx.value();
  const double goal_y = gy.value();

  // 3. 检查是否有已提交的逃脱点（状态保持）
  if (has_committed_) {
    const uint8_t ccost = getCost(committed_x_, committed_y_);
    const double cdx = robot_x - committed_x_;
    const double cdy = robot_y - committed_y_;
    const double cdist = std::sqrt(cdx * cdx + cdy * cdy);

    if (ccost < 235 && cdist < MAX_COMMIT_DISTANCE) {
      // 已提交点仍然有效且距离合理，继续返回同一个点
      outputResult(committed_x_, committed_y_);
      return BT::NodeStatus::SUCCESS;
    }
    // 已提交点失效（被障碍覆盖或距离太远），重新搜索
    has_committed_ = false;
    if (node_) {
      RCLCPP_INFO(node_->get_logger(),
        "[FindEscapePoint] Committed point (%.2f,%.2f) invalidated (cost=%d, dist=%.2f)",
        committed_x_, committed_y_, ccost, cdist);
    }
  }

  // 4. 计算后退方向 = normalize(robot - goal)
  double retreat_dx = robot_x - goal_x;
  double retreat_dy = robot_y - goal_y;
  const double retreat_len = std::sqrt(retreat_dx * retreat_dx + retreat_dy * retreat_dy);
  if (retreat_len > 1e-6) {
    retreat_dx /= retreat_len;
    retreat_dy /= retreat_len;
  } else {
    retreat_dx = 1.0;
    retreat_dy = 0.0;
  }

  // 5. 三阶段搜索
  const double step = 0.15;  // 搜索步长

  // 阶段①: cost<50, 0.5~2.0m, 优先后退
  SearchParams phase1{0.5, 2.0, step, 50, true};
  double ex = 0.0, ey = 0.0;
  if (searchPhase(robot_x, robot_y, goal_x, goal_y, retreat_dx, retreat_dy, phase1, ex, ey)) {
    commitAndOutput(ex, ey, "Phase1", robot_x, robot_y);
    return BT::NodeStatus::SUCCESS;
  }

  // 阶段②: cost<150, 0.5~3.0m, 优先后退
  SearchParams phase2{0.5, 3.0, step, 150, true};
  if (searchPhase(robot_x, robot_y, goal_x, goal_y, retreat_dx, retreat_dy, phase2, ex, ey)) {
    commitAndOutput(ex, ey, "Phase2", robot_x, robot_y);
    return BT::NodeStatus::SUCCESS;
  }

  // 阶段③: cost<235, 0.5~3.0m, 任意方向
  SearchParams phase3{0.5, 3.0, step, 235, false};
  if (searchPhase(robot_x, robot_y, goal_x, goal_y, retreat_dx, retreat_dy, phase3, ex, ey)) {
    commitAndOutput(ex, ey, "Phase3", robot_x, robot_y);
    return BT::NodeStatus::SUCCESS;
  }

  // 三阶段都没找到
  if (node_) {
    RCLCPP_WARN(node_->get_logger(),
      "[FindEscapePoint] No escape point found from (%.2f,%.2f) toward goal (%.2f,%.2f)",
      robot_x, robot_y, goal_x, goal_y);
  }
  return BT::NodeStatus::FAILURE;
}

void FindEscapePointAction::outputResult(double x, double y)
{
  setOutput("escape_x", x);
  setOutput("escape_y", y);

  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = 0.0;
  pose.pose.orientation.w = 1.0;
  setOutput("escape_pose", pose);
}

void FindEscapePointAction::commitAndOutput(
  double x, double y, const char * phase, double robot_x, double robot_y)
{
  has_committed_ = true;
  committed_x_ = x;
  committed_y_ = y;
  if (node_) {
    RCLCPP_INFO(node_->get_logger(),
      "[FindEscapePoint] %s: robot(%.2f,%.2f) → escape(%.2f,%.2f) [committed]",
      phase, robot_x, robot_y, x, y);
  }
  outputResult(x, y);
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::FindEscapePointAction, "FindEscapePoint");
