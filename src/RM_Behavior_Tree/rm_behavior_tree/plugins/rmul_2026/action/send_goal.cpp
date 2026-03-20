#include "rm_behavior_tree/plugins/rmul_2026/action/send_goal.hpp"
#include <rclcpp/rclcpp.hpp>

namespace rm_behavior_tree
{

SendGoalAction::SendGoalAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
  : BT::SyncActionNode(name, conf), node_(params.nh)
{
}

bool SendGoalAction::isSameGoal_(const geometry_msgs::msg::PoseStamped & a,
                                const geometry_msgs::msg::PoseStamped & b) const
{
  // Small tolerances to avoid spamming due to tiny floating noise.
  constexpr double kPosEps = 1e-3;
  auto absd = [](double x) { return std::fabs(x); };

  // Only compare position fields. If any position is NaN/Inf, treat as different.
  const double vals_a[] = {a.pose.position.x, a.pose.position.y, a.pose.position.z};
  const double vals_b[] = {b.pose.position.x, b.pose.position.y, b.pose.position.z};
  for (size_t i = 0; i < 3; ++i) {
    if (!isFinite_(vals_a[i]) || !isFinite_(vals_b[i])) {
      return false;
    }
  }

  if (absd(a.pose.position.x - b.pose.position.x) > kPosEps) return false;
  if (absd(a.pose.position.y - b.pose.position.y) > kPosEps) return false;
  if (absd(a.pose.position.z - b.pose.position.z) > kPosEps) return false;

  return true;
}

BT::NodeStatus SendGoalAction::tick()
{
  std::string topic_name = "goal_pose";
  getInput("topic_name", topic_name);
  if(!publisher_ || prev_topic_name_ != topic_name)
  {
    publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(topic_name, 1);
    prev_topic_name_ = topic_name;
  }

  geometry_msgs::msg::PoseStamped goal;
  std::string frame_id = "map";
  {
    auto r_frame = getInput<std::string>("frame_id");
    if (r_frame && !r_frame.value().empty()) {
      frame_id = r_frame.value();
    }
  }

  auto res = getInput<geometry_msgs::msg::PoseStamped>("goal_pose");
  if (res.has_value()) {
    goal = res.value();
  } else {
    double gx = 0.0, gy = 0.0;
    auto r_gx = getInput<double>("goal_x");
    auto r_gy = getInput<double>("goal_y");
    const bool has_x = r_gx.has_value();
    const bool has_y = r_gy.has_value();
    if (has_x && has_y) {
      gx = r_gx.value();
      gy = r_gy.value();
      goal.pose.position.x = gx;
      goal.pose.position.y = gy;
      goal.pose.position.z = 0.0;
      goal.pose.orientation.x = 0.0;
      goal.pose.orientation.y = 0.0;
      goal.pose.orientation.z = 0.0;
      goal.pose.orientation.w = 1.0;
    } else {
      RCLCPP_DEBUG(node_->get_logger(), "no goal_pose and goal_x/goal_y not provided");
      return BT::NodeStatus::FAILURE;
    }
  }

  if (!goal.header.frame_id.empty()) {
    frame_id = goal.header.frame_id;
  }

  int min_interval_ms = 0;
  auto r_interval = getInput<int>("min_interval_ms");
  if (r_interval) {
    min_interval_ms = r_interval.value();
    if (min_interval_ms < 0) {
      min_interval_ms = 0;
    }
  }

  auto now = node_->get_clock()->now();

  if (min_interval_ms > 0 && has_last_) {
    const bool same_goal = isSameGoal_(goal, last_goal_);
    if (same_goal) {
      const int64_t dt_ns = (now - last_pub_time_).nanoseconds();
      const int64_t min_dt_ns = static_cast<int64_t>(min_interval_ms) * 1000000LL;
      // If clock jumps backward OR interval not reached, skip publish
      if (dt_ns < 0 || dt_ns < min_dt_ns) {
        // Return SUCCESS even if we skip publishing! This prevents resetting the BT branch!
        return BT::NodeStatus::SUCCESS;
      }
    }
  }

  geometry_msgs::msg::PoseStamped msg;
  {
    const uint64_t ns = now.nanoseconds();
    msg.header.stamp.sec = static_cast<int32_t>(ns / 1000000000ULL);
    msg.header.stamp.nanosec = static_cast<uint32_t>(ns % 1000000000ULL);
  }
  msg.header.frame_id = frame_id;
  msg.pose.position.x = goal.pose.position.x;
  msg.pose.position.y = goal.pose.position.y;
  msg.pose.position.z = goal.pose.position.z;
  msg.pose.orientation.x = 0.0;
  msg.pose.orientation.y = 0.0;
  msg.pose.orientation.z = 0.0;
  msg.pose.orientation.w = 1.0;

  RCLCPP_INFO(
    node_->get_logger(),
    "Goal position: [ %.3f, %.3f, %.3f ]",
    goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);

  last_goal_ = goal;
  last_pub_time_ = now;
  has_last_ = true;

  publisher_->publish(msg);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::SendGoalAction, "SendGoal");
