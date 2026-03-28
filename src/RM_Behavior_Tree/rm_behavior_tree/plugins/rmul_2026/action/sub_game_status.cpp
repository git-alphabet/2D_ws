#include "rm_behavior_tree/plugins/rmul_2026/action/sub_game_status.hpp"

namespace rm_behavior_tree
{

SubGameStatusAction::SubGameStatusAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: BT::RosTopicSubNode<sp_msgs::msg::RMUL>(name, conf, params)
{
}

BT::NodeStatus SubGameStatusAction::onTick(
  const std::shared_ptr<sp_msgs::msg::RMUL> & last_msg)
{
  if (last_msg)  // empty if no new message received, since the last tick
  {
    RCLCPP_INFO_THROTTLE(
      logger(), *node_->get_clock(), 2000,
      "[%s] game_progress=%d stage_remain_time=%d current_hp=%d is_attacked=%d", name().c_str(),
      static_cast<int>(last_msg->game_progress), static_cast<int>(last_msg->stage_remain_time),
      static_cast<int>(last_msg->current_hp), static_cast<int>(last_msg->is_attacked));
    setOutput("game_status", *last_msg);
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::SubGameStatusAction, "SubGameStatus");