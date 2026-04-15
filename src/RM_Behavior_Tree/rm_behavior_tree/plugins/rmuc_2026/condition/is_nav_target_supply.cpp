#include "rm_behavior_tree/plugins/rmuc_2026/condition/is_nav_target_supply.hpp"

namespace rm_behavior_tree
{

IsNavTargetSupplyCondition::IsNavTargetSupplyCondition(
  const std::string & name,
  const BT::NodeConfig & conf,
  const BT::RosNodeParams & params)
: BT::SyncActionNode(name, conf)
{
  node_ = params.nh;
  if (!node_) {
    throw BT::RuntimeError("IsNavTargetSupply: failed to obtain ROS node from params.nh");
  }

  std::string topic = "goal_pose";
  getInput("topic_name", topic);

  sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    topic, rclcpp::QoS(1),
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      std::lock_guard<std::mutex> lk(mtx_);
      last_goal_ = *msg;
      has_goal_ = true;
    });

  RCLCPP_INFO(node_->get_logger(),
    "[IsNavTargetSupply:%s] subscribed to '%s'", name.c_str(), topic.c_str());
}

BT::NodeStatus IsNavTargetSupplyCondition::tick()
{
  if (!has_goal_.load()) {
    return BT::NodeStatus::FAILURE;
  }

  double gx = 0, gy = 0, radius = 1.5;
  getInput("goal_x", gx);
  getInput("goal_y", gy);
  getInput("arrive_radius", radius);

  double nav_x, nav_y;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    nav_x = last_goal_.pose.position.x;
    nav_y = last_goal_.pose.position.y;
  }

  double dx = nav_x - gx;
  double dy = nav_y - gy;
  bool match = (dx * dx + dy * dy) < (radius * radius);

  return match ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::IsNavTargetSupplyCondition, "IsNavTargetSupply");
