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
    last_pose_x_ = static_cast<double>(last_msg->pose_x);
    last_pose_y_ = static_cast<double>(last_msg->pose_y);
    last_is_at_nav_goal_ = last_msg->is_at_nav_goal;
    has_last_position_ = true;
  }

  if (has_last_position_) {
    setOutput("pose_x", last_pose_x_);
    setOutput("pose_y", last_pose_y_);
  }
  setOutput("is_at_nav_goal", last_is_at_nav_goal_);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

CreateRosNodePlugin(rm_behavior_tree::RmucSubRobotPositionAction, "RmucSubRobotPosition");
