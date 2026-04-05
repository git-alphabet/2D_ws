#include "rm_behavior_tree/plugins/rmuc_2026/condition/is_combat_allowed.hpp"

namespace rm_behavior_tree
{

IsCombatAllowedCondition::IsCombatAllowedCondition(
  const std::string & name, const BT::NodeConfig & conf)
: BT::ConditionNode(name, conf) {}

BT::NodeStatus IsCombatAllowedCondition::tick()
{
  int ammo = 300, heat = 0, heat_high = 210, hp = 400, hp_low = 180;
  getInput("ammo_allow", ammo);
  getInput("heat_cur", heat);
  getInput("heat_high", heat_high);
  getInput("hp_cur", hp);
  getInput("hp_low", hp_low);

  if (ammo <= 0) return BT::NodeStatus::FAILURE;
  if (heat >= heat_high) return BT::NodeStatus::FAILURE;
  if (hp < hp_low) return BT::NodeStatus::FAILURE;
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::IsCombatAllowedCondition>("IsCombatAllowed");
}
