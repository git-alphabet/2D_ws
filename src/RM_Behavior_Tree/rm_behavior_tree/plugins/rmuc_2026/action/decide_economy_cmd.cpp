#include "rm_behavior_tree/plugins/rmuc_2026/action/decide_economy_cmd.hpp"

namespace rm_behavior_tree
{

DecideEconomyCmdAction::DecideEconomyCmdAction(
  const std::string & name, const BT::NodeConfig & conf)
: BT::SyncActionNode(name, conf) {}

BT::NodeStatus DecideEconomyCmdAction::tick()
{
  int hp = 400, hp_max = 400, ammo = 300, ammo_target = 300, ammo_low = 80;
  bool disengaged = false, can_heal = false, can_ammo = false, base_threat = false;
  int coins = 0, remain_s = 420;

  getInput("hp_cur", hp);
  getInput("hp_max", hp_max);
  getInput("ammo_allow", ammo);
  getInput("ammo_target", ammo_target);
  getInput("ammo_low", ammo_low);
  getInput("is_disengaged", disengaged);
  getInput("can_remote_heal", can_heal);
  getInput("can_remote_ammo", can_ammo);
  getInput("team_coins", coins);
  getInput("stage_remain_time", remain_s);
  getInput("base_threat", base_threat);

  int trig_ammo = 0, trig_hp = 0, big_energy = 0;
  int allow_ammo = 0;
  getInput("allow_ammo_target_in", allow_ammo);

  // 远程回血：脱战且血量<50% 且金币充足
  if (disengaged && can_heal && hp_max > 0 && hp < hp_max / 2) {
    trig_hp = 1;
  }

  // 远程补弹：弹量不足且金币充足
  if (can_ammo && ammo < ammo_low) {
    trig_ammo = 1;
  }

  // 允许弹丸配额 (非负整数，累加)
  if (ammo < ammo_target && coins >= 100) {
    allow_ammo += 50;  // 每次请求 50 发
  }

  // 大能量机关：比赛中期 (120-300s elapsed → remain 120-300) 且基地受威胁
  if (base_threat && remain_s < 300 && remain_s > 120) {
    big_energy = 1;
  }

  setOutput("trigger_remote_ammo", trig_ammo);
  setOutput("trigger_remote_hp", trig_hp);
  setOutput("enable_big_energy", big_energy);
  setOutput("allow_ammo_target_out", allow_ammo);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::DecideEconomyCmdAction>("DecideEconomyCmd");
}
