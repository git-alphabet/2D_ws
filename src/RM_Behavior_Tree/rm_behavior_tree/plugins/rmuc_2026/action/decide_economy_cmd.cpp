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

  // ── 补给点兑换 (bit2-12, 单调递增) ──
  // 规则: 10金币/10发, 需在补给区/基地增益点/前哨站 (裁判系统侧判定位置)
  int allow_ammo = 0;
  getInput("allow_ammo_target_in", allow_ammo);

  int allow_ammo_max = 400, exchange_unit = 10, exchange_cost = 10;
  int interval_ms = 1000;
  getInput("allow_ammo_max", allow_ammo_max);
  getInput("exchange_unit", exchange_unit);
  getInput("exchange_cost", exchange_cost);
  getInput("ammo_increase_interval_ms", interval_ms);

  auto now = std::chrono::steady_clock::now();
  if (ammo < ammo_target && coins >= exchange_cost && allow_ammo < allow_ammo_max) {
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
      now - last_exchange_time_).count();
    if (last_exchange_time_.time_since_epoch().count() == 0 ||
        elapsed >= interval_ms) {
      allow_ammo += exchange_unit;
      if (allow_ammo > allow_ammo_max) allow_ammo = allow_ammo_max;
      last_exchange_time_ = now;
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
