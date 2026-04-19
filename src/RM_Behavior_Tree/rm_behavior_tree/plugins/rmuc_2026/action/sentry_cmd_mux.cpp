#include "rm_behavior_tree/plugins/rmuc_2026/action/sentry_cmd_mux.hpp"

namespace rm_behavior_tree
{

RmucSentryCmdMuxAction::RmucSentryCmdMuxAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: BT::RosTopicPubNode<sp_msgs::msg::RMUCSentryCmd>(name, conf, params)
{
}

bool RmucSentryCmdMuxAction::setMessage(sp_msgs::msg::RMUCSentryCmd & msg)
{
  int posture = 3;
  int confirm_respawn = 0, confirm_instant = 0;

  getInput("posture", posture);
  getInput("confirm_respawn", confirm_respawn);
  getInput("confirm_instant_respawn", confirm_instant);

  msg.cmd_posture = static_cast<uint8_t>(posture);
  msg.cmd_confirm_respawn = (confirm_respawn != 0);
  msg.cmd_confirm_instant_respawn = (confirm_instant != 0);
  // cmd_allow_ammo_target 保持默认 0 — 不再花金币兑换弹丸

  return true;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::RmucSentryCmdMuxAction, "SentryCmdMux");
