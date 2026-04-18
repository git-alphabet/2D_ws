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

  int movement_s = 5;
  getInput("forced_movement_s", movement_s);

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

  // ── 强制切换模式 (防御 or 移动) ──
  if (in_forced_switch_) {
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      now - forced_switch_start_).count();

    if (elapsed_ms >= forced_duration_ms_) {
      // 强制切换结束，放行正常姿态
      in_forced_switch_ = false;
      active_posture_ = desired;
      setOutput("final_posture", desired);
      std::cout << "[PostureDegradationGuard] 强制姿态 " << forced_posture_
                << " 结束，恢复姿态 " << desired << std::endl;
    } else {
      // 仍在强制切换中
      active_posture_ = forced_posture_;
      setOutput("final_posture", forced_posture_);
    }
    return BT::NodeStatus::SUCCESS;
  }

  // ── 检查降级 ──
  int64_t threshold_ms = static_cast<int64_t>(threshold_s) * 1000;
  if (desired >= 1 && desired <= kPostureCount &&
      cumulative_ms_[desired - 1] >= threshold_ms)
  {
    // 重置该姿态的累计计时器
    cumulative_ms_[desired - 1] = 0;

    // 特殊情况：防御(2)超时 → 强制移动(3)
    // 普通情况：攻击(1)/移动(3)超时 → 强制防御(2)
    if (desired == 2) {
      forced_posture_ = 3;  // 移动
      forced_duration_ms_ = movement_s * 1000;
    } else {
      forced_posture_ = 2;  // 防御
      forced_duration_ms_ = defense_s * 1000;
    }

    in_forced_switch_ = true;
    forced_switch_start_ = now;
    degraded_posture_ = desired;

    active_posture_ = forced_posture_;
    setOutput("final_posture", forced_posture_);

    std::cout << "[PostureDegradationGuard] 姿态 " << desired
              << " 累计超过 " << threshold_s << "s，强制切换到姿态 "
              << forced_posture_ << " 持续 "
              << (forced_duration_ms_ / 1000) << "s，计时器已重置" << std::endl;
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
