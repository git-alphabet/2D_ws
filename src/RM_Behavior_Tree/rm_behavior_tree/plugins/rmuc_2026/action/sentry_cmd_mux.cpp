#include "rm_behavior_tree/plugins/rmuc_2026/action/sentry_cmd_mux.hpp"

namespace rm_behavior_tree
{

RmucSentryCmdMuxAction::RmucSentryCmdMuxAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: BT::RosTopicPubNode<rm_decision_interfaces::msg::RMUCSentryCmd>(name, conf, params)
{
}

bool RmucSentryCmdMuxAction::setMessage(rm_decision_interfaces::msg::RMUCSentryCmd & msg)
{
  int posture = 3;
  int confirm_respawn = 0, confirm_instant = 0;
  int allow_ammo = 0, trig_ammo = 0, trig_hp = 0, big_energy = 0;

  getInput("posture", posture);
  getInput("confirm_respawn", confirm_respawn);
  getInput("confirm_instant_respawn", confirm_instant);
  getInput("allow_ammo_target", allow_ammo);
  getInput("trigger_remote_ammo", trig_ammo);
  getInput("trigger_remote_hp", trig_hp);
  getInput("enable_big_energy", big_energy);

  msg.cmd_posture = static_cast<uint8_t>(posture);
  msg.cmd_confirm_respawn = (confirm_respawn != 0);
  msg.cmd_confirm_instant_respawn = (confirm_instant != 0);
  msg.cmd_allow_ammo_target = static_cast<uint16_t>(allow_ammo);
  msg.cmd_trigger_remote_ammo = (trig_ammo != 0);
  msg.cmd_trigger_remote_hp = (trig_hp != 0);
  msg.cmd_enable_big_energy = (big_energy != 0);

  return true;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::RmucSentryCmdMuxAction, "SentryCmdMux");
