#include "rm_behavior_tree/plugins/rmuc_2026/action/fire_burst.hpp"

namespace rm_behavior_tree
{

FireBurstAction::FireBurstAction(
  const std::string & name, const BT::NodeConfig & conf)
: BT::StatefulActionNode(name, conf) {}

BT::NodeStatus FireBurstAction::onStart()
{
  getInput("burst_ms", burst_ms_);
  getInput("pause_ms", pause_ms_);
  start_time_ = std::chrono::steady_clock::now();
  firing_ = true;
  // TODO: 发送 fire_enable = true 到 RobotControl
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus FireBurstAction::onRunning()
{
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - start_time_).count();

  if (firing_) {
    if (elapsed >= burst_ms_) {
      firing_ = false;
      start_time_ = std::chrono::steady_clock::now();
      // TODO: 发送 fire_enable = false
    }
    return BT::NodeStatus::RUNNING;
  } else {
    // 暂停阶段
    if (elapsed >= pause_ms_) {
      return BT::NodeStatus::SUCCESS;  // 一轮射击完成
    }
    return BT::NodeStatus::RUNNING;
  }
}

void FireBurstAction::onHalted()
{
  firing_ = false;
  // TODO: 确保停火
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::FireBurstAction>("FireBurst");
}
