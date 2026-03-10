#include "rm_behavior_tree/plugins/rmuc_2026/condition/is_fire_window_ok.hpp"

namespace rm_behavior_tree
{

IsFireWindowOkCondition::IsFireWindowOkCondition(
  const std::string & name, const BT::NodeConfig & conf)
: BT::ConditionNode(name, conf) {}

BT::NodeStatus IsFireWindowOkCondition::tick()
{
  int heat = 0, heat_high = 210, ammo = 300;
  bool weak = false;
  getInput("heat_cur", heat);
  getInput("heat_high", heat_high);
  getInput("ammo_allow", ammo);
  getInput("is_weak", weak);

  if (weak || ammo <= 0 || heat >= heat_high) return BT::NodeStatus::FAILURE;
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::IsFireWindowOkCondition>("IsFireWindowOk");
}
