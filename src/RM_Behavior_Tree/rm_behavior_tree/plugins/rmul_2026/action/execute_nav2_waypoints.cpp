
#include "rm_behavior_tree/plugins/rmul_2026/action/execute_nav2_waypoints.hpp"
#include "behaviortree_ros2/plugins.hpp"

namespace rm_behavior_tree
{

ExecuteNav2Waypoints::ExecuteNav2Waypoints(
  const std::string& name,
  const BT::NodeConfig& conf,
  const BT::RosNodeParams& params)
: BT::RosActionNode<nav2_msgs::action::FollowWaypoints>(name, conf, params)
{
  
  rclcpp::NodeOptions options;
  if (node_->has_parameter("use_sim_time")) {
      options.parameter_overrides({{"use_sim_time", node_->get_parameter("use_sim_time").as_bool()}});
  }
  sub_node_ = std::make_shared<rclcpp::Node>("execute_nav2_waypoints_sub", node_->get_namespace(), options);
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(sub_node_);

  sub_ = sub_node_->create_subscription<visualization_msgs::msg::MarkerArray>(
    "waypoints", 10,
    [this](const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      if (completed_) return;

      std::vector<geometry_msgs::msg::PoseStamped> temp_waypoints;
      for (const auto& marker : msg->markers) {
        if (marker.action == visualization_msgs::msg::Marker::ADD && marker.type == visualization_msgs::msg::Marker::ARROW) {
          geometry_msgs::msg::PoseStamped pose;
          pose.header = marker.header;
          pose.pose = marker.pose;
          temp_waypoints.push_back(pose);
        }
      }
      
      if (!temp_waypoints.empty()) {
        waypoints_ = temp_waypoints;
        RCLCPP_INFO(sub_node_->get_logger(), "Intercepted /waypoints update! Current count: %zu", waypoints_.size());
      }
    });

  spin_thread_ = std::thread([this]() {
    executor_->spin();
  });
}

ExecuteNav2Waypoints::~ExecuteNav2Waypoints()
{
  if (executor_) {
    executor_->cancel();
  }
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
}

bool ExecuteNav2Waypoints::setGoal(Goal& goal)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (completed_ || waypoints_.empty()) {
    RCLCPP_WARN(node_->get_logger(), "ExecuteNav2Waypoints: No waypoints available or already completed. Skipping.");
    return false;
  }
  
  goal.poses = waypoints_;
  RCLCPP_INFO(node_->get_logger(), "Executing %zu waypoints by sending to action server...", waypoints_.size());
  return true;
}

BT::NodeStatus ExecuteNav2Waypoints::onResultReceived(const WrappedResult& wr)
{
  RCLCPP_INFO(node_->get_logger(), "ExecuteNav2Waypoints completed. Handing over to combat logic.");
  completed_ = true;
  return BT::NodeStatus::FAILURE; // 返回FAILURE以使得Fallback节点继续往下走向战斗逻辑
}

BT::NodeStatus ExecuteNav2Waypoints::onFailure(BT::ActionNodeErrorCode error)
{
  RCLCPP_WARN(node_->get_logger(), "ExecuteNav2Waypoints failed or skipped (Code %d). Handing over to combat logic.", static_cast<int>(error));
  return BT::NodeStatus::FAILURE;
}

} // namespace

CreateRosNodePlugin(rm_behavior_tree::ExecuteNav2Waypoints, "ExecuteNav2Waypoints");
