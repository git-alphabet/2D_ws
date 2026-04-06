#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__CANCEL_NAV_GOAL_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__CANCEL_NAV_GOAL_HPP_

#include <string>
#include <chrono>
#include <memory>
#include <iostream>
#include <iomanip>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "action_msgs/srv/cancel_goal.hpp"

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"

namespace rm_behavior_tree
{

class CancelNavGoalAction : public BT::SyncActionNode
{
public:
  CancelNavGoalAction(
    const std::string & name,
    const BT::NodeConfig & conf,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts()
  {
    return {
      // 你在 XML 里填 "navigate_to_pose" 或 "/navigate_to_pose" 都可以
      BT::InputPort<std::string>("action_name", "navigate_to_pose", "action name or namespace"),

      // 可选：节流（ms）。避免频繁 cancel spam；=0 表示不节流
      BT::InputPort<int>("min_interval_ms", 0, "throttle interval ms"),

      // 可选：wait_for_service 的等待时间（ms）
      // =0 表示只做一次非阻塞检查（service 不在就直接按 fail_on_unavailable 决策）
      BT::InputPort<int>("wait_for_service_ms", 0, "wait for service ms"),

      // 可选：是否在 service 不可用时返回 FAILURE
      // 默认 false：不阻塞树（没 nav 可取消时当作成功）
      BT::InputPort<bool>("fail_on_unavailable", false, "fail if service unavailable"),

      // 可选：等待响应的时间（ms），=0 不等待（纯 fire-and-forget）
      BT::InputPort<int>("wait_response_ms", 0, "wait response ms"),

      // 可选：输出 cancel service 的 return_code（如果等待到了响应）
      BT::OutputPort<int>("return_code")
    };
  }

  BT::NodeStatus tick() override;

private:
  static std::string makeCancelServiceName_(std::string action_name);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<action_msgs::srv::CancelGoal>::SharedPtr client_;
  std::string service_name_;

  rclcpp::Time last_call_time_{0, 0, RCL_SYSTEM_TIME};
  bool has_last_call_{false};
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__CANCEL_NAV_GOAL_HPP_
