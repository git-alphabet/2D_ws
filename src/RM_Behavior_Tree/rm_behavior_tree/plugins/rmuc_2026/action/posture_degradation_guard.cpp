#include "rm_behavior_tree/plugins/rmuc_2026/action/posture_degradation_guard.hpp"
#include <iostream>

namespace rm_behavior_tree
{

PostureDegradationGuard::PostureDegradationGuard(
  const std::string & name, const BT::NodeConfig & conf)
: BT::SyncActionNode(name, conf) {}

BT::NodeStatus PostureDegradationGuard::tick()
{
  int desired = 3;
  getInput("desired_posture", desired);

  int threshold_s = 180;
  getInput("degradation_threshold_s", threshold_s);

  int defense_s = 5;
  getInput("forced_defense_s", defense_s);

  auto now = std::chrono::steady_clock::now();

  // 首次初始化
  if (!initialized_) {
    initialized_ = true;
    active_posture_ = desired;
    last_tick_time_ = now;
    setOutput("final_posture", desired);
    return BT::NodeStatus::SUCCESS;
  }

  // 累加当前活跃姿态的时间
  auto delta_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    now - last_tick_time_).count();
  last_tick_time_ = now;

  if (active_posture_ >= 1 && active_posture_ <= kPostureCount) {
    cumulative_ms_[active_posture_ - 1] += delta_ms;
  }

  // ── 强制防御模式 ──
  if (in_forced_defense_) {
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      now - forced_defense_start_).count();

    if (elapsed_ms >= static_cast<int64_t>(defense_s) * 1000) {
      // 强制防御结束，放行正常姿态
      in_forced_defense_ = false;
      active_posture_ = desired;
      setOutput("final_posture", desired);
      std::cout << "[PostureDegradationGuard] 强制防御结束，恢复姿态 " << desired << std::endl;
    } else {
      // 仍在强制防御中
      active_posture_ = 2;  // 防御
      setOutput("final_posture", 2);
    }
    return BT::NodeStatus::SUCCESS;
  }

  // ── 检查降级 ──
  int64_t threshold_ms = static_cast<int64_t>(threshold_s) * 1000;
  if (desired >= 1 && desired <= kPostureCount &&
      cumulative_ms_[desired - 1] >= threshold_ms)
  {
    // 触发强制防御，并重置该姿态的累计计时器
    in_forced_defense_ = true;
    forced_defense_start_ = now;
    degraded_posture_ = desired;
    cumulative_ms_[desired - 1] = 0;  // 重置！卡 timing

    active_posture_ = 2;  // 防御
    setOutput("final_posture", 2);

    std::cout << "[PostureDegradationGuard] 姿态 " << desired
              << " 累计超过 " << threshold_s << "s，强制防御 "
              << defense_s << "s，计时器已重置" << std::endl;
    return BT::NodeStatus::SUCCESS;
  }

  // ── 正常放行 ──
  active_posture_ = desired;
  setOutput("final_posture", desired);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::PostureDegradationGuard>("PostureDegradationGuard");
}
