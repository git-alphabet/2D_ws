#include "rm_behavior_tree/plugins/rmuc_2026/action/hold_for_supply_ammo_tick.hpp"

namespace rm_behavior_tree
{

HoldForSupplyAmmoTickAction::HoldForSupplyAmmoTickAction(
  const std::string & name, const BT::NodeConfig & conf)
: BT::StatefulActionNode(name, conf) {}

BT::NodeStatus HoldForSupplyAmmoTickAction::onStart()
{
  return onRunning();
}

BT::NodeStatus HoldForSupplyAmmoTickAction::onRunning()
{
  int ammo = 0, target = 300;
  getInput("ammo_allow", ammo);
  getInput("ammo_target", target);

  if (ammo >= target) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

void HoldForSupplyAmmoTickAction::onHalted() {}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::HoldForSupplyAmmoTickAction>("HoldForSupplyAmmoTick");
}
