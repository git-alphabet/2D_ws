#include "rm_behavior_tree/plugins/rmul_2026/action/sub_robot_status.hpp"

namespace rm_behavior_tree
{

SubRobotStatusAction::SubRobotStatusAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: BT::RosTopicSubNode<sp_msgs::msg::RMUL>(name, conf, params)
{
}

BT::NodeStatus SubRobotStatusAction::onTick(
  const std::shared_ptr<sp_msgs::msg::RMUL> & last_msg)
{
  if (last_msg)  // empty if no new message received, since the last tick
  {
    RCLCPP_DEBUG(
      logger(), "[%s] new message, shooter_heat: %s", name().c_str(),
      std::to_string(last_msg->shooter_heat).c_str());
    setOutput("robot_status", last_msg);
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::SubRobotStatusAction, "SubRobotStatus");