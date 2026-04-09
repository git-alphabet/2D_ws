#include "rm_behavior_tree/plugins/rmuc_2026/action/decide_economy_cmd.hpp"

namespace rm_behavior_tree
{

DecideEconomyCmdAction::DecideEconomyCmdAction(
  const std::string & name, const BT::NodeConfig & conf)
: BT::SyncActionNode(name, conf) {}

BT::NodeStatus DecideEconomyCmdAction::tick()
{
  int hp = 400, hp_max = 400, ammo = 300, ammo_target = 300, ammo_low = 80;
  bool can_heal = false, can_ammo = false;
  int coins = 0, remain_s = 420;

  getInput("hp_cur", hp);
  getInput("hp_max", hp_max);
  getInput("ammo_allow", ammo);
  getInput("ammo_target", ammo_target);
  getInput("ammo_low", ammo_low);
  bool is_disengaged = false;
  getInput("is_disengaged", is_disengaged);
  getInput("can_remote_heal", can_heal);
  getInput("can_remote_ammo", can_ammo);
  getInput("team_coins", coins);
  getInput("stage_remain_time", remain_s);

  int trig_ammo = 0, trig_hp = 0;
  int allow_ammo = 0;
  getInput("allow_ammo_target_in", allow_ammo);

  // 远程回血：必须脱战 + can_remote_heal + 血量<50%
  if (is_disengaged && can_heal && hp_max > 0 && hp < hp_max / 2) {
    trig_hp = 1;
  }

  // 远程补弹：必须脱战 + 弹量不足且金币充足
  if (is_disengaged && can_ammo && ammo < ammo_low) {
    trig_ammo = 1;
  }

  // 允许弹丸配额 (非负整数，累加，有上限，带限速)
  int allow_ammo_max = 400;
  int interval_ms = 1000;
  getInput("allow_ammo_max", allow_ammo_max);
  getInput("ammo_increase_interval_ms", interval_ms);
  auto now = std::chrono::steady_clock::now();
  if (ammo < ammo_target && coins >= 100 && allow_ammo < allow_ammo_max) {
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
      now - last_ammo_increase_time_).count();
    if (last_ammo_increase_time_.time_since_epoch().count() == 0 ||
        elapsed >= interval_ms) {
      allow_ammo += 50;
      if (allow_ammo > allow_ammo_max) allow_ammo = allow_ammo_max;
      last_ammo_increase_time_ = now;
    }
  }

  setOutput("trigger_remote_ammo", trig_ammo);
  setOutput("trigger_remote_hp", trig_hp);
  setOutput("allow_ammo_target_out", allow_ammo);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::DecideEconomyCmdAction>("DecideEconomyCmd");
}
