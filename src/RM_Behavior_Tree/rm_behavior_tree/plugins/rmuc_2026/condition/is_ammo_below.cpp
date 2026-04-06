#include "rm_behavior_tree/plugins/rmuc_2026/condition/is_ammo_below.hpp"

namespace rm_behavior_tree
{

IsAmmoBelowCondition::IsAmmoBelowCondition(
  const std::string & name, const BT::NodeConfig & conf)
: BT::ConditionNode(name, conf) {}

BT::NodeStatus IsAmmoBelowCondition::tick()
{
  int ammo = 300, low = 80;
  getInput("ammo_allow", ammo);
  getInput("ammo_low", low);
  return (ammo < low) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::IsAmmoBelowCondition>("IsAmmoBelow");
}
