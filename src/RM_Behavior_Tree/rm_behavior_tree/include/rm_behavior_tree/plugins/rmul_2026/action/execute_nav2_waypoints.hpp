
#pragma once

#include "behaviortree_ros2/bt_action_node.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <mutex>

namespace rm_behavior_tree
{

class ExecuteNav2Waypoints : public BT::RosActionNode<nav2_msgs::action::FollowWaypoints>
{
public:
  ExecuteNav2Waypoints(const std::string& name,
                       const BT::NodeConfig& conf,
                       const BT::RosNodeParams& params);

  ~ExecuteNav2Waypoints() override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({});
  }

  bool setGoal(Goal& goal) override;

  BT::NodeStatus onResultReceived(const WrappedResult& wr) override;

  virtual BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;

private:
  rclcpp::Node::SharedPtr sub_node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr sub_;
  std::thread spin_thread_;
  std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
  std::mutex mutex_;
  bool completed_ = false;
};

} // namespace
