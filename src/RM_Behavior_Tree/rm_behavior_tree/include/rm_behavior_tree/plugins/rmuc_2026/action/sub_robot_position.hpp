#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__SUB_ROBOT_POSITION_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__SUB_ROBOT_POSITION_HPP_

#include <mutex>
#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "rm_decision_interfaces/msg/rmuc_robot_position.hpp"

namespace rm_behavior_tree
{
class RmucSubRobotPositionAction : public BT::SyncActionNode
{
public:
  RmucSubRobotPositionAction(
    const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic_name", "/robot_position", "订阅的话题名"),
      BT::OutputPort<double>("pose_x"),
      BT::OutputPort<double>("pose_y"),
      BT::OutputPort<double>("pose_yaw"),
      BT::OutputPort<bool>("is_at_nav_goal")};
  }

  BT::NodeStatus tick() override;

private:
  void callback(const rm_decision_interfaces::msg::RMUCRobotPosition::SharedPtr msg);
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<rm_decision_interfaces::msg::RMUCRobotPosition>::SharedPtr sub_;
  mutable std::mutex mutex_;
  double pose_x_{0.0}, pose_y_{0.0}, pose_yaw_{0.0};
  bool is_at_nav_goal_{false};
  bool has_data_{false};
};
}  // namespace rm_behavior_tree

#endif
