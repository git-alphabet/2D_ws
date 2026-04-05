#include "rm_behavior_tree/plugins/rmuc_2026/action/robot_control.hpp"

namespace rm_behavior_tree
{

RmucRobotControlAction::RmucRobotControlAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: RosTopicPubNode<rm_decision_interfaces::msg::RMUCRobotControl>(name, conf, params)
{
}

bool RmucRobotControlAction::setMessage(rm_decision_interfaces::msg::RMUCRobotControl & msg)
{
  msg.stop_gimbal_scan = false;
  msg.chassis_spin = false;

  getInput("stop_gimbal_scan", msg.stop_gimbal_scan);
  getInput("chassis_spin", msg.chassis_spin);

  return true;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::RmucRobotControlAction, "RmucRobotControl");
