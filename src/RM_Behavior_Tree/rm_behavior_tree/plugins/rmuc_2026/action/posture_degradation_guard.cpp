#include "rm_behavior_tree/plugins/rmuc_2026/action/posture_degradation_guard.hpp"
#include "behaviortree_cpp/blackboard.h"
#include <cmath>
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

  bool is_dead = false;
  bool need_heal_recovery = false;
  bool base_threat = false;
  bool semantic_zone_active = false;
  bool nav_goal_valid = false;
  double pose_x = 0.0;
  double pose_y = 0.0;
  double nav_goal_x = 0.0;
  double nav_goal_y = 0.0;
  double fortress_area_x = 0.0;
  double fortress_area_y = 0.0;
  double arrive_radius = 0.5;
  int hp_cur = 10000;
  int hp_low = 0;
  int ammo_allow = 300;
  int ammo_low = 80;
  getInput("is_dead", is_dead);
  getInput("need_heal_recovery", need_heal_recovery);
  getInput("hp_cur", hp_cur);
  getInput("hp_low", hp_low);
  getInput("ammo_allow", ammo_allow);
  getInput("ammo_low", ammo_low);
  getInput("base_threat", base_threat);
  getInput("semantic_zone_active", semantic_zone_active);
  getInput("nav_goal_valid", nav_goal_valid);
  getInput("pose_x", pose_x);
  getInput("pose_y", pose_y);
  getInput("nav_goal_x", nav_goal_x);
  getInput("nav_goal_y", nav_goal_y);
  getInput("fortress_area_x", fortress_area_x);
  getInput("fortress_area_y", fortress_area_y);
  getInput("arrive_radius", arrive_radius);
  if (auto * root_bb = config().blackboard->rootBlackboard()) {
    bool threshold_overridden = false;
    try {
      threshold_overridden = root_bb->get<bool>("supply.threshold_overridden");
    } catch (...) {
      threshold_overridden = false;
    }
    if (threshold_overridden) {
      try {
        ammo_low = root_bb->get<int>("supply.next_threshold");
      } catch (...) {
        // Keep the XML-provided fallback threshold.
      }
    }
  }

  int threshold_s = 180;
  getInput("degradation_threshold_s", threshold_s);

  int defense_s = 5;
  getInput("forced_defense_s", defense_s);

  int movement_s = 5;
  getInput("forced_movement_s", movement_s);

  auto now = std::chrono::steady_clock::now();

  int effective_desired = desired;
  const bool bypass_degradation =
    is_dead || need_heal_recovery || hp_cur < hp_low || ammo_allow < ammo_low || base_threat;

  // 首次初始化
  if (!initialized_) {
    initialized_ = true;
    active_posture_ = effective_desired;
    last_tick_time_ = now;
  } else {
    // 累加当前活跃姿态的时间
    auto delta_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      now - last_tick_time_).count();
    last_tick_time_ = now;

    if (active_posture_ >= 1 && active_posture_ <= kPostureCount) {
      cumulative_ms_[active_posture_ - 1] += delta_ms;
    }
  }

  if (bypass_degradation) {
    if (in_forced_switch_) {
      std::cout << "[PostureDegradationGuard] 高优先级状态接管，取消强制姿态 "
                << forced_posture_ << "，放行姿态 " << effective_desired << std::endl;
    }
    in_forced_switch_ = false;
    forced_posture_ = 0;
    forced_duration_ms_ = 0;
    active_posture_ = effective_desired;
    setOutput("final_posture", effective_desired);
    return BT::NodeStatus::SUCCESS;
  }

  if (semantic_zone_active) {
    if (in_forced_switch_) {
      std::cout << "[PostureDegradationGuard] 语义区接管，取消强制姿态 "
                << forced_posture_ << "，强制移动姿态" << std::endl;
    }
    in_forced_switch_ = false;
    forced_posture_ = 0;
    forced_duration_ms_ = 0;
    active_posture_ = 3;
    setOutput("final_posture", 3);
    return BT::NodeStatus::SUCCESS;
  }

  const double fortress_goal_dist = std::hypot(nav_goal_x - fortress_area_x, nav_goal_y - fortress_area_y);
  const double goal_arrive_dist = std::hypot(pose_x - nav_goal_x, pose_y - nav_goal_y);
  const bool fortress_standby =
    nav_goal_valid && fortress_goal_dist <= 0.05 && goal_arrive_dist <= arrive_radius;

  if (fortress_standby) {
    if (in_forced_switch_) {
      std::cout << "[PostureDegradationGuard] 堡垒区站定禁用姿态降级，取消强制姿态 "
                << forced_posture_ << "，放行姿态 " << effective_desired << std::endl;
    }
    in_forced_switch_ = false;
    forced_posture_ = 0;
    forced_duration_ms_ = 0;
    active_posture_ = effective_desired;
    setOutput("final_posture", effective_desired);
    return BT::NodeStatus::SUCCESS;
  }

  // ── 强制切换模式 (防御 or 移动) ──
  if (in_forced_switch_) {
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      now - forced_switch_start_).count();

    if (elapsed_ms >= forced_duration_ms_) {
      // 强制切换结束，放行正常姿态
      in_forced_switch_ = false;
      active_posture_ = effective_desired;
      setOutput("final_posture", effective_desired);
      std::cout << "[PostureDegradationGuard] 强制姿态 " << forced_posture_
                << " 结束，恢复姿态 " << effective_desired << std::endl;
    } else {
      // 仍在强制切换中
      active_posture_ = forced_posture_;
      setOutput("final_posture", forced_posture_);
    }
    return BT::NodeStatus::SUCCESS;
  }

  // ── 检查降级 ──
  int64_t threshold_ms = static_cast<int64_t>(threshold_s) * 1000;
  if (effective_desired >= 1 && effective_desired <= kPostureCount &&
      cumulative_ms_[effective_desired - 1] >= threshold_ms)
  {
    // 重置该姿态的累计计时器
    cumulative_ms_[effective_desired - 1] = 0;

    // 特殊情况：防御(2)超时 → 强制移动(3)
    // 普通情况：攻击(1)/移动(3)超时 → 强制防御(2)
    if (effective_desired == 2) {
      forced_posture_ = 3;  // 移动
      forced_duration_ms_ = movement_s * 1000;
    } else {
      forced_posture_ = 2;  // 防御
      forced_duration_ms_ = defense_s * 1000;
    }

    in_forced_switch_ = true;
    forced_switch_start_ = now;
    degraded_posture_ = effective_desired;

    active_posture_ = forced_posture_;
    setOutput("final_posture", forced_posture_);

    std::cout << "[PostureDegradationGuard] 姿态 " << effective_desired
              << " 累计超过 " << threshold_s << "s，强制切换到姿态 "
              << forced_posture_ << " 持续 "
              << (forced_duration_ms_ / 1000) << "s，计时器已重置" << std::endl;
    return BT::NodeStatus::SUCCESS;
  }

  // ── 正常放行 ──
  active_posture_ = effective_desired;
  setOutput("final_posture", effective_desired);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::PostureDegradationGuard>("PostureDegradationGuard");
}
