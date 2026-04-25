#include "rm_behavior_tree/plugins/rmuc_2026/action/select_posture.hpp"
#include <iostream>

namespace rm_behavior_tree
{

SelectPostureAction::SelectPostureAction(
  const std::string & name, const BT::NodeConfig & conf)
: BT::SyncActionNode(name, conf) {}

BT::NodeStatus SelectPostureAction::tick()
{
  int desired = 3;
  getInput("posture_value", desired);

  if (desired < 1 || desired > 3) {
    std::cerr << "[SelectPosture] 无效姿态值 " << desired << "，强制使用 3(移动)" << std::endl;
    desired = 3;
  }

  auto now = std::chrono::steady_clock::now();

  // 首次初始化
  if (!initialized_) {
    active_posture_ = desired;
    last_switch_time_ = now;
    initialized_ = true;
    setOutput("posture_out", active_posture_);
    setOutput("is_allow_select_posture", true);
    std::cout << "[SelectPosture:" << name() << "] 初始化 → posture=" << active_posture_ << std::endl;
    return BT::NodeStatus::SUCCESS;
  }

  // 请求的姿态与当前一致 → 保持，允许发送
  if (desired == active_posture_) {
    setOutput("posture_out", active_posture_);
    setOutput("is_allow_select_posture", true);
    return BT::NodeStatus::SUCCESS;
  }

  // 请求切换 → 检查冷却
  auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    now - last_switch_time_).count();

  if (elapsed_ms >= kCooldownMs) {
    // 冷却已过，允许切换
    active_posture_ = desired;
    last_switch_time_ = now;
    setOutput("posture_out", active_posture_);
    setOutput("is_allow_select_posture", true);
    std::cout << "[SelectPosture:" << name() << "] 切换 → posture=" << active_posture_
              << " (cooldown " << elapsed_ms << "ms)" << std::endl;
  } else {
    // 冷却中，拒绝切换，保持当前姿态，不允许发送新指令
    setOutput("posture_out", active_posture_);
    setOutput("is_allow_select_posture", false);
    std::cout << "[SelectPosture:" << name() << "] 冷却中拒绝切换 desired="
              << desired << " active=" << active_posture_
              << " remain=" << (kCooldownMs - elapsed_ms) << "ms" << std::endl;
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::SelectPostureAction>("SelectPosture");
}
