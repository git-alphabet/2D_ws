#include "rm_behavior_tree/plugins/rmuc_2026/action/sub_robot_position.hpp"
#include "behaviortree_ros2/plugins.hpp"

namespace rm_behavior_tree
{

RmucSubRobotPositionAction::RmucSubRobotPositionAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: BT::RosTopicSubNode<sp_msgs::msg::RMUCRobotPosition>(name, conf, params)
{
}

BT::NodeStatus RmucSubRobotPositionAction::onTick(
  const std::shared_ptr<sp_msgs::msg::RMUCRobotPosition> & last_msg)
{
  if (last_msg) {
    setOutput("pose_x", static_cast<double>(last_msg->pose_x));
    setOutput("pose_y", static_cast<double>(last_msg->pose_y));
    setOutput("pose_yaw", static_cast<double>(last_msg->pose_yaw));
    setOutput("is_at_nav_goal", last_msg->is_at_nav_goal);
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

CreateRosNodePlugin(rm_behavior_tree::RmucSubRobotPositionAction, "RmucSubRobotPosition");
