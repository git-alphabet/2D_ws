#include "rm_behavior_tree/plugins/rmul_2026/action/execute_prepared_route.hpp"

#include <cmath>

#include "behaviortree_ros2/plugins.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rm_behavior_tree
{

ExecutePreparedRouteAction::ExecutePreparedRouteAction(
  const std::string & name,
  const BT::NodeConfig & conf,
  const BT::RosNodeParams &)
: BT::SyncActionNode(name, conf)
{
}

void ExecutePreparedRouteAction::loadWaypointsFromPorts()
{
  waypoints_.clear();

  auto push_if_enabled = [&](const char * en, const char * x, const char * y) {
    bool enabled = false;
    double px = 0.0;
    double py = 0.0;
    getInput(en, enabled);
    getInput(x, px);
    getInput(y, py);
    if (enabled) {
      waypoints_.push_back({px, py});
    }
  };

  push_if_enabled("wpt0_enable", "wpt0_x", "wpt0_y");
  push_if_enabled("wpt1_enable", "wpt1_x", "wpt1_y");
  push_if_enabled("wpt2_enable", "wpt2_x", "wpt2_y");

  initialized_ = true;
}

BT::NodeStatus ExecutePreparedRouteAction::tick()
{
  bool enable = false;
  getInput("enable", enable);
  if (!enable) {
    initialized_ = false;
    completed_ = false;
    current_index_ = 0;
    return BT::NodeStatus::FAILURE;
  }

  if (!initialized_) {
    loadWaypointsFromPorts();
  }

  if (waypoints_.empty()) {
    return BT::NodeStatus::FAILURE;
  }

  if (completed_) {
    return BT::NodeStatus::FAILURE;
  }

  auto pose_x = getInput<double>("pose_x");
  auto pose_y = getInput<double>("pose_y");
  if (!pose_x || !pose_y) {
    return BT::NodeStatus::FAILURE;
  }

  double arrive_radius = 0.5;
  std::string frame_id = "map";
  getInput("arrive_radius", arrive_radius);
  getInput("frame_id", frame_id);
  if (arrive_radius < 0.05) {
    arrive_radius = 0.05;
  }

  const auto & current = waypoints_[current_index_];
  const double distance = std::hypot(current.x - pose_x.value(), current.y - pose_y.value());

  if (distance <= arrive_radius) {
    current_index_++;
    if (current_index_ >= waypoints_.size()) {
      completed_ = true;
      RCLCPP_WARN(
        rclcpp::get_logger("ExecutePreparedRoute"),
        "prepared route finished, hand over to normal BT decision");
      return BT::NodeStatus::FAILURE;
    }
  }

  const auto & target = waypoints_[current_index_];
  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header.frame_id = frame_id;
  goal_pose.pose.position.x = target.x;
  goal_pose.pose.position.y = target.y;
  goal_pose.pose.position.z = 0.0;
  goal_pose.pose.orientation.w = 1.0;

  setOutput("goal_x", target.x);
  setOutput("goal_y", target.y);
  setOutput("goal_pose", goal_pose);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

CreateRosNodePlugin(rm_behavior_tree::ExecutePreparedRouteAction, "ExecutePreparedRoute");
