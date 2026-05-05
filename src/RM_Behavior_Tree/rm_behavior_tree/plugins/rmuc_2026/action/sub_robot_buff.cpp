#include "rm_behavior_tree/plugins/rmuc_2026/action/sub_robot_buff.hpp"

namespace rm_behavior_tree
{

RmucSubRobotBuffAction::RmucSubRobotBuffAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: BT::RosTopicSubNode<sp_msgs::msg::RMUCRobotBuff>(name, conf, params)
{
}

BT::NodeStatus RmucSubRobotBuffAction::onTick(
  const std::shared_ptr<sp_msgs::msg::RMUCRobotBuff> & last_msg)
{
  if (last_msg) {
    RCLCPP_DEBUG(
      logger(), "[%s] robot_buff: vulnerability_pct=%u",
      name().c_str(), last_msg->vulnerability_pct);
    setOutput("robot_buff", *last_msg);
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::RmucSubRobotBuffAction, "RmucSubRobotBuff");
