#include "rm_behavior_tree/plugins/rmuc_2026/action/sub_robot_status.hpp"

namespace rm_behavior_tree
{

RmucSubRobotStatusAction::RmucSubRobotStatusAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: BT::RosTopicSubNode<rm_decision_interfaces::msg::RMUCRobotStatus>(name, conf, params)
{
}

BT::NodeStatus RmucSubRobotStatusAction::onTick(
  const std::shared_ptr<rm_decision_interfaces::msg::RMUCRobotStatus> & last_msg)
{
  if (last_msg) {
    RCLCPP_DEBUG(
      logger(), "[%s] new RMUC msg, shooter_heat: %d", name().c_str(),
      last_msg->shooter_heat);
    setOutput("robot_status", last_msg);
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::RmucSubRobotStatusAction, "RmucSubRobotStatus");
