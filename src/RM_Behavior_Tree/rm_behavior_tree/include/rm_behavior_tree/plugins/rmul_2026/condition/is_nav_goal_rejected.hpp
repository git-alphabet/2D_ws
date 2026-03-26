#ifndef RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_NAV_GOAL_REJECTED_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_NAV_GOAL_REJECTED_HPP_

#include <chrono>
#include <mutex>
#include <string>

#include "action_msgs/msg/goal_status_array.hpp"
#include "behaviortree_cpp/condition_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rm_behavior_tree
{

/**
 * @brief 检测 Nav2 导航目标是否被拒绝/中止
 *
 * 订阅 navigate_to_pose/_action/status，当最新 goal 状态为
 * ABORTED(6) 时返回 SUCCESS（表示"是，被拒绝了"），否则 FAILURE。
 *
 * 锁存机制：一旦检测到 ABORTED，持续返回 SUCCESS，直到：
 *   - 出现新的 ACCEPTED/EXECUTING 状态（说明新目标已被接受）
 *   - 或超过 latch_timeout_ms 自动解除
 */
class IsNavGoalRejectedCondition : public BT::ConditionNode
{
public:
  IsNavGoalRejectedCondition(
    const std::string & name,
    const BT::NodeConfig & conf,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("action_name", "navigate_to_pose",
        "Nav2 action name to monitor"),
      BT::InputPort<int>("latch_timeout_ms", 10000,
        "auto-unlatch after this many ms (0=never auto-unlatch)"),
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr status_sub_;
  std::string subscribed_topic_;

  mutable std::mutex mutex_;
  int8_t latest_status_{0};  // 0=UNKNOWN

  // 锁存
  bool rejected_latched_{false};
  std::chrono::steady_clock::time_point latch_time_;

  void statusCallback(const action_msgs::msg::GoalStatusArray::SharedPtr msg);
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_NAV_GOAL_REJECTED_HPP_
