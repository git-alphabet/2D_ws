#include "rm_behavior_tree/plugins/rmul_2026/condition/is_nav_goal_rejected.hpp"

#include "behaviortree_ros2/plugins.hpp"

namespace rm_behavior_tree
{

IsNavGoalRejectedCondition::IsNavGoalRejectedCondition(
  const std::string & name,
  const BT::NodeConfig & conf,
  const BT::RosNodeParams & params)
: BT::ConditionNode(name, conf)
{
  node_ = params.nh;
}

void IsNavGoalRejectedCondition::statusCallback(
  const action_msgs::msg::GoalStatusArray::SharedPtr msg)
{
  if (!msg || msg->status_list.empty()) {
    return;
  }
  // 取最新的 goal 状态（列表末尾）
  std::lock_guard<std::mutex> lock(mutex_);
  latest_status_ = msg->status_list.back().status;
}

BT::NodeStatus IsNavGoalRejectedCondition::tick()
{
  // 1. 确保订阅就绪
  std::string action_name = "navigate_to_pose";
  getInput("action_name", action_name);
  const std::string topic = action_name + "/_action/status";

  if (!status_sub_ || subscribed_topic_ != topic) {
    if (node_) {
      status_sub_ = node_->create_subscription<action_msgs::msg::GoalStatusArray>(
        topic, rclcpp::QoS(1),
        [this](const action_msgs::msg::GoalStatusArray::SharedPtr msg) {
          statusCallback(msg);
        });
      subscribed_topic_ = topic;
    }
  }

  if (node_) {
    rclcpp::spin_some(node_);
  }

  // 2. 读取最新状态
  int8_t current_status;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    current_status = latest_status_;
  }

  // GoalStatus 常量:
  // STATUS_UNKNOWN = 0, STATUS_ACCEPTED = 1, STATUS_EXECUTING = 2,
  // STATUS_CANCELING = 3, STATUS_SUCCEEDED = 4, STATUS_CANCELED = 5,
  // STATUS_ABORTED = 6
  constexpr int8_t STATUS_ACCEPTED = 1;
  constexpr int8_t STATUS_EXECUTING = 2;
  constexpr int8_t STATUS_SUCCEEDED = 4;
  constexpr int8_t STATUS_ABORTED = 6;

  // 3. 如果新目标已被接受/执行中/成功，解除锁存
  if (current_status == STATUS_ACCEPTED ||
      current_status == STATUS_EXECUTING ||
      current_status == STATUS_SUCCEEDED)
  {
    if (rejected_latched_) {
      rejected_latched_ = false;
      if (node_) {
        RCLCPP_INFO(node_->get_logger(),
          "[IsNavGoalRejected] New goal accepted/executing, unlatch");
      }
    }
    return BT::NodeStatus::FAILURE;  // 未被拒绝
  }

  // 4. 检测 ABORTED → 锁存
  if (current_status == STATUS_ABORTED) {
    if (!rejected_latched_) {
      rejected_latched_ = true;
      latch_time_ = std::chrono::steady_clock::now();
      if (node_) {
        RCLCPP_WARN(node_->get_logger(),
          "[IsNavGoalRejected] Nav goal ABORTED detected, latching");
      }
    }
  }

  // 5. 锁存超时自动解除
  if (rejected_latched_) {
    int latch_timeout_ms = 10000;
    getInput("latch_timeout_ms", latch_timeout_ms);

    if (latch_timeout_ms > 0) {
      const auto elapsed = std::chrono::steady_clock::now() - latch_time_;
      const auto elapsed_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
      if (elapsed_ms >= latch_timeout_ms) {
        rejected_latched_ = false;
        if (node_) {
          RCLCPP_INFO(node_->get_logger(),
            "[IsNavGoalRejected] Latch timeout (%dms), auto-unlatch",
            latch_timeout_ms);
        }
        return BT::NodeStatus::FAILURE;
      }
    }
    return BT::NodeStatus::SUCCESS;  // 是，被拒绝了
  }

  return BT::NodeStatus::FAILURE;  // 未被拒绝
}

}  // namespace rm_behavior_tree

CreateRosNodePlugin(rm_behavior_tree::IsNavGoalRejectedCondition, "IsNavGoalRejected");
