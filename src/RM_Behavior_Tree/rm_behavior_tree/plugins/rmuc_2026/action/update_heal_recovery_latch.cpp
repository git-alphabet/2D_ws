#include "rm_behavior_tree/plugins/rmuc_2026/action/update_heal_recovery_latch.hpp"

#include <chrono>
#include <cstdio>

namespace rm_behavior_tree
{

UpdateHealRecoveryLatchAction::UpdateHealRecoveryLatchAction(
  const std::string & name, const BT::NodeConfig & conf)
: BT::SyncActionNode(name, conf) {}

BT::NodeStatus UpdateHealRecoveryLatchAction::tick()
{
  int hp_cur = 0;
  int hp_low = 180;
  int hp_safe = 280;
  getInput("hp_cur", hp_cur);
  getInput("hp_low", hp_low);
  getInput("hp_safe", hp_safe);

  bool need_heal_recovery = false;
  int low_hp_supply_retreat_count = 0;
  auto * root_bb = config().blackboard->rootBlackboard();
  if (root_bb) {
    try {
      need_heal_recovery = root_bb->get<bool>("need_heal_recovery");
    } catch (const std::exception &) {
      need_heal_recovery = false;
    }
    try {
      low_hp_supply_retreat_count = root_bb->get<int>("low_hp_supply_retreat_count");
    } catch (const std::exception &) {
      low_hp_supply_retreat_count = 0;
    }
  }

  const bool was_need_heal_recovery = need_heal_recovery;
  if (hp_cur < hp_low) {
    need_heal_recovery = true;
  } else if (hp_cur >= hp_safe) {
    need_heal_recovery = false;
  }

  if (!was_need_heal_recovery && need_heal_recovery) {
    ++low_hp_supply_retreat_count;
  }

  const auto now = std::chrono::steady_clock::now();
  if (
    last_monitor_print_time_ == std::chrono::steady_clock::time_point{} ||
    now - last_monitor_print_time_ >= std::chrono::minutes(1))
  {
    last_monitor_print_time_ = now;
    std::printf(
      "[UpdateHealRecoveryLatch] low HP supply retreat monitor: count=%d active=%d hp=%d low=%d safe=%d\n",
      low_hp_supply_retreat_count, need_heal_recovery ? 1 : 0, hp_cur, hp_low, hp_safe);
  }

  if (root_bb) {
    root_bb->set("need_heal_recovery", need_heal_recovery);
    root_bb->set("low_hp_supply_retreat_count", low_hp_supply_retreat_count);
  }
  setOutput("need_heal_recovery", need_heal_recovery);
  setOutput("low_hp_supply_retreat_count", low_hp_supply_retreat_count);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::UpdateHealRecoveryLatchAction>(
    "UpdateHealRecoveryLatch");
}
