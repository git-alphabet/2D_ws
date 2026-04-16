#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__SELECT_POSTURE_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__SELECT_POSTURE_HPP_

#include <string>
#include <chrono>
#include "behaviortree_cpp/action_node.h"

namespace rm_behavior_tree
{
/// 带防抖的姿态选择器：通过 XML 端口指定姿态值 (1=进攻 2=防御 3=移动)
/// 防抖规则：每次姿态至少维持 4 秒才允许切换
/// 输出 is_allow_select_posture 门控：true 时才允许下游发送 sentry_cmd
class SelectPostureAction : public BT::SyncActionNode
{
public:
  SelectPostureAction(const std::string & name, const BT::NodeConfig & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("posture_value", "3", "目标姿态 (1=进攻 2=防御 3=移动)"),
      BT::OutputPort<int>("posture_out", "输出到黑板的姿态值"),
      BT::OutputPort<bool>("is_allow_select_posture", "姿态切换门控 (true=允许发送)")};
  }

  BT::NodeStatus tick() override;

private:
  static constexpr int64_t kCooldownMs = 5000;
  int active_posture_{0};
  std::chrono::steady_clock::time_point last_switch_time_{};
  bool initialized_{false};
};
}  // namespace rm_behavior_tree

#endif
