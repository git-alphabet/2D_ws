#include "rm_behavior_tree/plugins/rmuc_2026/action/sub_robot_position.hpp"
#include "behaviortree_ros2/plugins.hpp"

namespace rm_behavior_tree
{

RmucSubRobotPositionAction::RmucSubRobotPositionAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: BT::SyncActionNode(name, conf), node_(params.nh)
{
  if (!node_) {
    throw std::runtime_error("RmucSubRobotPositionAction: ROS node is null");
  }
  std::string topic;
  getInput<std::string>("topic_name", topic);

  rclcpp::QoS qos(10);
  qos.reliable();
  sub_ = node_->create_subscription<rm_decision_interfaces::msg::RMUCRobotPosition>(
    topic, qos,
    [this](const rm_decision_interfaces::msg::RMUCRobotPosition::SharedPtr msg) { this->callback(msg); });
}

void RmucSubRobotPositionAction::callback(
  const rm_decision_interfaces::msg::RMUCRobotPosition::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  pose_x_ = msg->pose_x;
  pose_y_ = msg->pose_y;
  pose_yaw_ = msg->pose_yaw;
  is_at_nav_goal_ = msg->is_at_nav_goal;
  has_data_ = true;
}

BT::NodeStatus RmucSubRobotPositionAction::tick()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_data_) {
    return BT::NodeStatus::SUCCESS;
  }
  setOutput("pose_x", pose_x_);
  setOutput("pose_y", pose_y_);
  setOutput("pose_yaw", pose_yaw_);
  setOutput("is_at_nav_goal", is_at_nav_goal_);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

CreateRosNodePlugin(rm_behavior_tree::RmucSubRobotPositionAction, "RmucSubRobotPosition");
