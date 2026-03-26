#include "rm_behavior_tree/plugins/rmul_2026/action/nav_control_cmd.hpp"

namespace rm_behavior_tree
{

NavControlCmdAction::NavControlCmdAction(
  const std::string & name,
  const BT::NodeConfig & conf,
  const BT::RosNodeParams & params)
: BT::RosTopicPubNode<sp_msgs::msg::RMUL>(name, conf, params)
{
}

bool NavControlCmdAction::setMessage(sp_msgs::msg::RMUL & msg)
{
  msg.cmd_type = 0;
  msg.emergency_stop = false;

  getInput("cmd_type", msg.cmd_type);
  getInput("emergency_stop", msg.emergency_stop);

  return true;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::NavControlCmdAction, "NavControlCmd");
