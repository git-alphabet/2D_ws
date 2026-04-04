#include "rm_behavior_tree/plugins/rmuc_2026/action/aim_at_target.hpp"

#include <sstream>

namespace rm_behavior_tree
{

AimAtTargetAction::AimAtTargetAction(
  const std::string & name, const BT::NodeConfig & conf,
  const BT::RosNodeParams & params)
: BT::StatefulActionNode(name, conf)
{
  node_ = params.nh;
  if (!node_) {
    throw BT::RuntimeError("AimAtTarget: params.nh is null");
  }
  pub_aim_ = node_->create_publisher<geometry_msgs::msg::PointStamped>("aim_target", 10);
}

BT::NodeStatus AimAtTargetAction::onStart()
{
  std::string target;
  if (!getInput("target", target) || target.empty()) {
    return BT::NodeStatus::FAILURE;
  }

  // 解析 "id:x:y" 格式
  std::replace(target.begin(), target.end(), ':', ' ');
  std::istringstream ss(target);
  if (!(ss >> target_id_ >> target_x_ >> target_y_)) {
    RCLCPP_WARN(node_->get_logger(), "AimAtTarget: failed to parse target '%s'", target.c_str());
    return BT::NodeStatus::FAILURE;
  }

  // 发布瞄准坐标到 /aim_target
  geometry_msgs::msg::PointStamped msg;
  msg.header.stamp = node_->now();
  msg.header.frame_id = "map";
  msg.point.x = target_x_;
  msg.point.y = target_y_;
  msg.point.z = 0.0;
  pub_aim_->publish(msg);

  RCLCPP_INFO(node_->get_logger(), "AimAtTarget: aiming at enemy %d (%.2f, %.2f)",
              target_id_, target_x_, target_y_);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus AimAtTargetAction::onRunning()
{
  // 假设云台响应足够快，直接返回 SUCCESS
  // 后续可接入云台反馈判断是否已对准
  return BT::NodeStatus::SUCCESS;
}

void AimAtTargetAction::onHalted()
{
  // 发布空坐标（z = -1 表示停止追踪）通知云台停止
  geometry_msgs::msg::PointStamped msg;
  msg.header.stamp = node_->now();
  msg.header.frame_id = "map";
  msg.point.x = 0.0;
  msg.point.y = 0.0;
  msg.point.z = -1.0;
  pub_aim_->publish(msg);
  RCLCPP_INFO(node_->get_logger(), "AimAtTarget: halted, stop tracking");
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::AimAtTargetAction, "AimAtTarget");
