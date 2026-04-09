#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__DECIDE_ECONOMY_CMD_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__DECIDE_ECONOMY_CMD_HPP_

#include <chrono>
#include <string>
#include "behaviortree_cpp/action_node.h"

namespace rm_behavior_tree
{
/// 决定经济指令：远程回血/远程补弹/允许弹丸配额
class DecideEconomyCmdAction : public BT::SyncActionNode
{
public:
  DecideEconomyCmdAction(const std::string & name, const BT::NodeConfig & conf);
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("hp_cur"),
      BT::InputPort<int>("hp_max"),
      BT::InputPort<int>("ammo_allow"),
      BT::InputPort<int>("ammo_target"),
      BT::InputPort<int>("ammo_low"),
      BT::InputPort<bool>("is_disengaged", "false", "is_disengaged"),
      BT::InputPort<bool>("can_remote_heal"),
      BT::InputPort<bool>("can_remote_ammo"),
      BT::InputPort<int>("team_coins"),
      BT::InputPort<int>("stage_remain_time"),
      BT::InputPort<int>("allow_ammo_max", 400, "允许发弹量上限"),
      BT::InputPort<int>("ammo_increase_interval_ms", 1000, "允许弹量累加最小间隔(ms)"),
      BT::InputPort<int>("allow_ammo_target_in"),
      BT::OutputPort<int>("allow_ammo_target_out"),
      BT::OutputPort<int>("trigger_remote_ammo"),
      BT::OutputPort<int>("trigger_remote_hp")};
  }
  BT::NodeStatus tick() override;

private:
  std::chrono::steady_clock::time_point last_ammo_increase_time_{};
};
}  // namespace rm_behavior_tree
#endif
