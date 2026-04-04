#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__HOLD_FOR_SUPPLY_AMMO_TICK_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__HOLD_FOR_SUPPLY_AMMO_TICK_HPP_

#include <string>
#include "behaviortree_cpp/action_node.h"

namespace rm_behavior_tree
{
/// 在补给区等待弹丸配额到账。RUNNING 直到 ammo_allow ≥ ammo_target
class HoldForSupplyAmmoTickAction : public BT::StatefulActionNode
{
public:
  HoldForSupplyAmmoTickAction(const std::string & name, const BT::NodeConfig & conf);
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("stage_elapsed_time"),
      BT::InputPort<int>("ammo_allow"),
      BT::InputPort<int>("ammo_target", "300", "ammo_target")};
  }
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
};
}  // namespace rm_behavior_tree
#endif
