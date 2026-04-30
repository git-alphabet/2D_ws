#include "rm_behavior_tree/plugins/rmuc_2026/action/update_heal_recovery_latch.hpp"

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
  auto * root_bb = config().blackboard->rootBlackboard();
  if (root_bb) {
    try {
      need_heal_recovery = root_bb->get<bool>("need_heal_recovery");
    } catch (const std::exception &) {
      need_heal_recovery = false;
    }
  }

  if (hp_cur < hp_low) {
    need_heal_recovery = true;
  } else if (hp_cur >= hp_safe) {
    need_heal_recovery = false;
  }

  if (root_bb) {
    root_bb->set("need_heal_recovery", need_heal_recovery);
  }
  setOutput("need_heal_recovery", need_heal_recovery);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::UpdateHealRecoveryLatchAction>(
    "UpdateHealRecoveryLatch");
}