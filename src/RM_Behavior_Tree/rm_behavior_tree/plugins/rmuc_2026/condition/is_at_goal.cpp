#include "rm_behavior_tree/plugins/rmuc_2026/condition/is_at_goal.hpp"
#include <iostream>
#include <type_traits>
#include "behaviortree_ros2/plugins.hpp"

namespace
{
// 全局共享 costmap 状态（同一 .so 内所有 IsAtGoal 实例共享）
struct CostmapState
{
  rclcpp::CallbackGroup::SharedPtr cb_group;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub;
  nav_msgs::msg::OccupancyGrid::SharedPtr data;
};
CostmapState g_costmap;
}  // namespace

namespace rm_behavior_tree
{

IsAtGoalCondition::IsAtGoalCondition(
  const std::string & name, const BT::NodeConfig & conf,
  const BT::RosNodeParams & params)
: BT::ConditionNode(name, conf)
{
  auto get_node = [](const auto & nh) -> rclcpp::Node::SharedPtr {
    using NH = std::decay_t<decltype(nh)>;
    if constexpr (std::is_convertible_v<NH, rclcpp::Node::SharedPtr>) {
      return nh;
    } else {
      return nh.lock();
    }
  };
  node_ = get_node(params.nh);
  if (!node_) {
    throw BT::RuntimeError("IsAtGoal: failed to obtain ROS node from params.nh");
  }
  costmap_topic_ = params.default_port_value;
  if (costmap_topic_.empty()) {
    costmap_topic_ = "global_costmap/costmap";
  }
}

void IsAtGoalCondition::ensureCostmapSub()
{
  if (g_costmap.sub) return;

  g_costmap.cb_group = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  g_costmap.executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  g_costmap.executor->add_callback_group(
    g_costmap.cb_group, node_->get_node_base_interface());

  rclcpp::SubscriptionOptions opts;
  opts.callback_group = g_costmap.cb_group;

  g_costmap.sub = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    costmap_topic_,
    rclcpp::QoS(1),
    [](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
      g_costmap.data = std::move(msg);
    },
    opts);

  std::cout << "[IsAtGoal] subscribed to costmap topic: " << costmap_topic_ << std::endl;
}

bool IsAtGoalCondition::hasLineOfSight(double x1, double y1, double x2, double y2)
{
  if (!g_costmap.data) return true;  // 无 costmap 数据 → 放行（降级为纯距离判断）

  const auto & info = g_costmap.data->info;
  const auto & grid = g_costmap.data->data;
  const double res = info.resolution;
  const double ox = info.origin.position.x;
  const double oy = info.origin.position.y;
  const int w = static_cast<int>(info.width);
  const int h = static_cast<int>(info.height);

  // 世界坐标 → 栅格坐标
  int gx1 = static_cast<int>((x1 - ox) / res);
  int gy1 = static_cast<int>((y1 - oy) / res);
  int gx2 = static_cast<int>((x2 - ox) / res);
  int gy2 = static_cast<int>((y2 - oy) / res);

  // Bresenham 直线算法：检查连线上是否有障碍物
  int dx = std::abs(gx2 - gx1);
  int dy = std::abs(gy2 - gy1);
  int sx = (gx1 < gx2) ? 1 : -1;
  int sy = (gy1 < gy2) ? 1 : -1;
  int err = dx - dy;

  // OccupancyGrid: 100 = lethal, 99 = inscribed, -1 = unknown
  constexpr int8_t kLethalThreshold = 90;

  int cx = gx1, cy = gy1;
  while (true) {
    if (cx >= 0 && cx < w && cy >= 0 && cy < h) {
      if (grid[cy * w + cx] >= kLethalThreshold) {
        return false;  // 连线上有障碍物
      }
    }
    if (cx == gx2 && cy == gy2) break;
    int e2 = 2 * err;
    if (e2 > -dy) { err -= dy; cx += sx; }
    if (e2 < dx) { err += dx; cy += sy; }
  }
  return true;  // 无障碍
}

BT::NodeStatus IsAtGoalCondition::tick()
{
  ensureCostmapSub();
  if (g_costmap.executor) {
    g_costmap.executor->spin_some();
  }

  double px = 0, py = 0, gx = 0, gy = 0, radius = 0.35;
  getInput("pose_x", px);
  getInput("pose_y", py);
  getInput("goal_x", gx);
  getInput("goal_y", gy);
  getInput("arrive_radius", radius);

  double dist = std::hypot(gx - px, gy - py);
  bool in_range = dist < radius;
  bool los = in_range ? hasLineOfSight(px, py, gx, gy) : false;
  bool at_goal = in_range && los;

  // 仅在本节点实例的状态变化时打印一次
  if (first_print_ || at_goal != last_at_goal_) {
    first_print_ = false;
    last_at_goal_ = at_goal;
    bool has_costmap = (g_costmap.data != nullptr);
    std::cout << "[IsAtGoal:" << name() << "] pose=(" << px << "," << py
              << ") goal=(" << gx << "," << gy
              << ") dist=" << dist << " radius=" << radius
              << " costmap=" << (has_costmap ? "yes" : "NO");
    if (in_range && !los) {
      std::cout << " BLOCKED_BY_WALL";
    }
    std::cout << " → " << (at_goal ? "SUCCESS" : "FAILURE") << std::endl;
  }

  return at_goal ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace rm_behavior_tree

CreateRosNodePlugin(rm_behavior_tree::IsAtGoalCondition, "IsAtGoal");
