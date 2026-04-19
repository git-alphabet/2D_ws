#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__HOLD_FOR_SUPPLY_AMMO_TICK_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__HOLD_FOR_SUPPLY_AMMO_TICK_HPP_

#include <string>
#include <chrono>
#include <algorithm>
#include "behaviortree_cpp/action_node.h"

namespace rm_behavior_tree
{
/// 在补给区等待弹丸配额到账（带30s超时和重试降级）
/// - 弹药增加 → SUCCESS，重置重试状态
/// - 30s超时 → SUCCESS（退出补弹），降级阈值供下次触发
class HoldForSupplyAmmoTickAction : public BT::StatefulActionNode
{
public:
  HoldForSupplyAmmoTickAction(const std::string & name, const BT::NodeConfig & conf);
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("ammo_allow"),
      BT::InputPort<int>("timeout_s", "30", "补给等待超时(秒)"),
      BT::InputPort<int>("default_threshold", "80", "默认弹药低阈值"),
      BT::BidirectionalPort<int>("supply_fail_count", 0, "补给失败计数"),
      BT::BidirectionalPort<int>("supply_next_threshold", 80, "下次触发补弹的弹药阈值")};
  }
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  int initial_ammo_{0};
  std::chrono::steady_clock::time_point start_time_;
};
}  // namespace rm_behavior_tree
#endif
