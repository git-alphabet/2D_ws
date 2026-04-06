#include "rm_behavior_tree/plugins/rmuc_2026/action/sub_game_status.hpp"

namespace rm_behavior_tree
{

RmucSubGameStatusAction::RmucSubGameStatusAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: BT::RosTopicSubNode<sp_msgs::msg::RMUCGameStatus>(name, conf, params)
{
}

BT::NodeStatus RmucSubGameStatusAction::onTick(
  const std::shared_ptr<sp_msgs::msg::RMUCGameStatus> & last_msg)
{
  if (last_msg) {
    RCLCPP_DEBUG(
      logger(), "[%s] new RMUC msg, game_progress: %d, remain: %d", name().c_str(),
      last_msg->game_progress, last_msg->stage_remain_time);
    setOutput("game_status", *last_msg);
    // 同步输出当前时间（毫秒）
    const auto now_ms = static_cast<std::uint64_t>(node_->now().nanoseconds() / 1000000ULL);
    setOutput("now_ms", now_ms);
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::RmucSubGameStatusAction, "RmucSubGameStatus");
