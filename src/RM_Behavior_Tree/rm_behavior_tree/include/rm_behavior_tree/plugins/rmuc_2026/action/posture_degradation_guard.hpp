#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__POSTURE_DEGRADATION_GUARD_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__POSTURE_DEGRADATION_GUARD_HPP_

#include <string>
#include <chrono>
#include <array>
#include "behaviortree_cpp/action_node.h"

namespace rm_behavior_tree
{
/// 姿态降级守卫：跟踪每个姿态的累计使用时间，超过阈值时强制切换姿态
/// 以"卡 timing"方式重置计时器，避免姿态效果降级
///
/// 工作流程:
///   1. 跟踪当前姿态的累计时间
///   2. 累计时间 > degradation_threshold_s → 强制切换姿态
///      - 攻击(1)/移动(3) → 强制防御(2) 持续 forced_defense_s 秒
///      - 防御(2) → 强制移动(3) 持续 forced_movement_s 秒 (特殊情况)
///   3. 强制切换期间重置该姿态的累计计时器
///   4. 切换结束后放行，恢复正常姿态选择
class PostureDegradationGuard : public BT::SyncActionNode
{
public:
  PostureDegradationGuard(const std::string & name, const BT::NodeConfig & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("desired_posture", "3", "期望姿态 (来自战术子树 SelectPosture)"),
      BT::InputPort<bool>("base_threat", "false", "基地威胁锁存状态"),
      BT::InputPort<double>("pose_x", "0.0", "当前 x"),
      BT::InputPort<double>("pose_y", "0.0", "当前 y"),
      BT::InputPort<double>("defend_anchor_x", "0.0", "基地防御锚点 x"),
      BT::InputPort<double>("defend_anchor_y", "0.0", "基地防御锚点 y"),
      BT::InputPort<double>("arrive_radius", "0.8", "到达防御锚点判定半径"),
      BT::InputPort<int>("degradation_threshold_s", "180", "姿态降级阈值 (秒，默认3分钟)"),
      BT::InputPort<int>("forced_defense_s", "5", "强制防御持续时间 (秒，匹配裁判系统冷却)"),
      BT::InputPort<int>("forced_movement_s", "5", "防御超时→强制移动持续时间 (秒)"),
      BT::OutputPort<int>("final_posture", "最终输出姿态 (可能被覆写为防御或移动)")};
  }

  BT::NodeStatus tick() override;

private:
  static constexpr int kPostureCount = 3;  // 1=攻击, 2=防御, 3=移动

  int active_posture_{3};
  bool initialized_{false};
  std::chrono::steady_clock::time_point last_tick_time_{};

  // 每个姿态的累计时间 (index: posture-1, 即 [0]=攻击 [1]=防御 [2]=移动)
  std::array<int64_t, kPostureCount> cumulative_ms_{0, 0, 0};

  // 强制切换状态 (防御→移动 或 非防御→防御)
  bool in_forced_switch_{false};
  int forced_posture_{0};     // 强制切换到的目标姿态 (2=防御, 3=移动)
  int forced_duration_ms_{0}; // 强制切换持续时间
  std::chrono::steady_clock::time_point forced_switch_start_{};
  int degraded_posture_{0};   // 触发降级的姿态，用于重置计时器
};
}  // namespace rm_behavior_tree

#endif
