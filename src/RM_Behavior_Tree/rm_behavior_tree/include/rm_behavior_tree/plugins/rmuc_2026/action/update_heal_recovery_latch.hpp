#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__UPDATE_HEAL_RECOVERY_LATCH_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__UPDATE_HEAL_RECOVERY_LATCH_HPP_

#include <chrono>
#include <string>

#include "behaviortree_cpp/action_node.h"

namespace rm_behavior_tree
{

class UpdateHealRecoveryLatchAction : public BT::SyncActionNode
{
public:
  UpdateHealRecoveryLatchAction(const std::string & name, const BT::NodeConfig & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("hp_cur"),
      BT::InputPort<int>("hp_low", "180", "hp_low"),
      BT::InputPort<int>("hp_safe", "280", "hp_safe"),
      BT::OutputPort<bool>("need_heal_recovery"),
      BT::BidirectionalPort<int>(
        "low_hp_supply_retreat_count", 0, "low hp triggered supply retreat count")};
  }

  BT::NodeStatus tick() override;

private:
  std::chrono::steady_clock::time_point last_monitor_print_time_{};
};

}  // namespace rm_behavior_tree

#endif
