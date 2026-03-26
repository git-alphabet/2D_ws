#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_ROBOT_POSITION_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_ROBOT_POSITION_HPP_

#include <mutex>
#include <string>
#include <memory>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"

#include "sp_msgs/msg/rmul.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace rm_behavior_tree
{

class SubRobotPositionAction : public BT::SyncActionNode
{
public:
  SubRobotPositionAction(
    const std::string & name,
    const BT::NodeConfig & conf,
    const BT::RosNodeParams & params);

  /// BehaviorTree 要求的静态端口声明
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic_name", std::string("red_standard_robot1"), "订阅的话题名"),
      BT::OutputPort<double>("pose_x"),
      BT::OutputPort<double>("pose_y")
    };
  }

  BT::NodeStatus tick() override;

private:
  void robot_position_callback(
    const sp_msgs::msg::RMUL::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sp_msgs::msg::RMUL>::SharedPtr sub_;
  
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  mutable std::mutex mutex_;

  // 最新一次接收到的数据
  double pose_x_{0.0};
  double pose_y_{0.0};
  rclcpp::Time last_stamp_;

  bool has_data_{false};
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_ROBOT_POSITION_HPP_
