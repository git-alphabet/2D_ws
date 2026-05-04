#include "rm_behavior_tree/plugins/rmuc_2026/action/nav_control_cmd.hpp"

namespace rm_behavior_tree
{

RmucNavControlCmdAction::RmucNavControlCmdAction(
  const std::string & name,
  const BT::NodeConfig & conf,
  const BT::RosNodeParams & params)
: BT::RosTopicPubNode<sp_msgs::msg::RMUCNavControlCmd>(name, conf, params)
{
}

bool RmucNavControlCmdAction::setMessage(sp_msgs::msg::RMUCNavControlCmd & msg)
{
  msg.cmd_type = 0;

  getInput("cmd_type", msg.cmd_type);

  return true;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::RmucNavControlCmdAction, "RmucNavControlCmd");
