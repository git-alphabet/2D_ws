#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__AIM_AT_TARGET_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__AIM_AT_TARGET_HPP_

#include <string>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"

namespace rm_behavior_tree
{
/// 接收 "id:x:y" 格式目标字符串，解析后向云台发送瞄准坐标
/// 通过 /aim_target 话题发布 geometry_msgs/PointStamped
class AimAtTargetAction : public BT::StatefulActionNode
{
public:
  AimAtTargetAction(
    const std::string & name, const BT::NodeConfig & conf,
    const BT::RosNodeParams & params);
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("target")};
  }
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_aim_;
  int target_id_{0};
  double target_x_{0.0};
  double target_y_{0.0};
};
}  // namespace rm_behavior_tree
#endif
