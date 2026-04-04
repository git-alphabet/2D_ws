#include "rm_behavior_tree/plugins/rmul_2026/action/robot_control.hpp"

namespace rm_behavior_tree
{

RobotControlAction::RobotControlAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: RosTopicPubNode<sp_msgs::msg::RMUL>(name, conf, params)
{
}

bool RobotControlAction::setMessage(sp_msgs::msg::RMUL & msg)
{
  msg.stop_gimbal_scan = false;
  msg.chassis_spin = false;

  getInput("stop_gimbal_scan", msg.stop_gimbal_scan);
  getInput("chassis_spin", msg.chassis_spin);

  // std::cout << "stop_gimbal_scan: " << msg.stop_gimbal_scan << '\n';
  // std::cout << "chassis_spin: " << msg.chassis_spin << '\n';

  return true;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::RobotControlAction, "RobotControl");