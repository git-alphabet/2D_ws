#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__DECIDE_ECONOMY_CMD_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__DECIDE_ECONOMY_CMD_HPP_

#include <chrono>
#include <string>
#include "behaviortree_cpp/action_node.h"

namespace rm_behavior_tree
{
/// 决定经济指令：补给点兑换(bit2-12, 10金币/10发)
class DecideEconomyCmdAction : public BT::SyncActionNode
{
public:
  DecideEconomyCmdAction(const std::string & name, const BT::NodeConfig & conf);
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("ammo_allow"),
      BT::InputPort<int>("ammo_target"),
      BT::InputPort<int>("team_coins"),
      BT::InputPort<int>("allow_ammo_max", 400, "补给点兑换上限(bit2-12)"),
      BT::InputPort<int>("exchange_unit", 10, "补给点最小兑换单位(10发)"),
      BT::InputPort<int>("exchange_cost", 10, "补给点兑换代价(10金币/10发)"),
      BT::InputPort<int>("ammo_increase_interval_ms", 1000, "兑换最小间隔(ms)"),
      BT::InputPort<int>("allow_ammo_target_in"),
      BT::OutputPort<int>("allow_ammo_target_out")};
  }
  BT::NodeStatus tick() override;

private:
  std::chrono::steady_clock::time_point last_exchange_time_{};
};
}  // namespace rm_behavior_tree
#endif
