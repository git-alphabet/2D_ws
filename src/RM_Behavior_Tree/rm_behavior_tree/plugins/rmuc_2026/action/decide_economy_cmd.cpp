#include "rm_behavior_tree/plugins/rmuc_2026/action/decide_economy_cmd.hpp"

namespace rm_behavior_tree
{

DecideEconomyCmdAction::DecideEconomyCmdAction(
  const std::string & name, const BT::NodeConfig & conf)
: BT::SyncActionNode(name, conf) {}

BT::NodeStatus DecideEconomyCmdAction::tick()
{
  int ammo = 300, ammo_target = 300;
  int coins = 0;

  getInput("ammo_allow", ammo);
  getInput("ammo_target", ammo_target);
  getInput("team_coins", coins);

  int allow_ammo = 0;
  getInput("allow_ammo_target_in", allow_ammo);

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

  setOutput("allow_ammo_target_out", allow_ammo);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::DecideEconomyCmdAction>("DecideEconomyCmd");
}
