#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__FIRE_BURST_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__FIRE_BURST_HPP_

#include <string>
#include <chrono>
#include "behaviortree_cpp/action_node.h"

namespace rm_behavior_tree
{
/// 执行一次射击脉冲：开火 burst_ms 毫秒 → 暂停 pause_ms 毫秒 → 返回 SUCCESS
class FireBurstAction : public BT::StatefulActionNode
{
public:
  FireBurstAction(const std::string & name, const BT::NodeConfig & conf);
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("burst_ms", "180", "burst_ms"),
      BT::InputPort<int>("pause_ms", "120", "pause_ms")};
  }
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::chrono::steady_clock::time_point start_time_;
  int burst_ms_{180};
  int pause_ms_{120};
  bool firing_{false};
};
}  // namespace rm_behavior_tree
#endif
