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
  int allow_ammo = 0, trig_ammo = 0, trig_hp = 0;

  getInput("posture", posture);
  getInput("confirm_respawn", confirm_respawn);
  getInput("confirm_instant_respawn", confirm_instant);
  getInput("allow_ammo_target", allow_ammo);
  getInput("trigger_remote_ammo", trig_ammo);
  getInput("trigger_remote_hp", trig_hp);

  msg.cmd_posture = static_cast<uint8_t>(posture);
  msg.cmd_confirm_respawn = (confirm_respawn != 0);
  msg.cmd_confirm_instant_respawn = (confirm_instant != 0);
  msg.cmd_allow_ammo_target = static_cast<uint16_t>(allow_ammo);

  // 远程兑换：上升沿检测 → 计数器+1（协议 bit13-16/bit17-20，4位单调递增）
  bool trig_ammo_now = (trig_ammo != 0);
  if (trig_ammo_now && !last_trig_ammo_ && remote_ammo_count_ < 15) {
    ++remote_ammo_count_;
  }
  last_trig_ammo_ = trig_ammo_now;

  bool trig_hp_now = (trig_hp != 0);
  if (trig_hp_now && !last_trig_hp_ && remote_hp_count_ < 15) {
    ++remote_hp_count_;
  }
  last_trig_hp_ = trig_hp_now;

  msg.cmd_trigger_remote_ammo = remote_ammo_count_;
  msg.cmd_trigger_remote_hp = remote_hp_count_;

  return true;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::RmucSentryCmdMuxAction, "SentryCmdMux");
