#include "rm_behavior_tree/plugins/rmul_2026/action/clear_recovery_flag.hpp"

namespace rm_behavior_tree
{

ClearRecoveryFlagAction::ClearRecoveryFlagAction(
  const std::string & name, const BT::NodeConfig & conf)
: BT::SyncActionNode(name, conf)
{
}

ClearRecoveryFlagAction::ClearRecoveryFlagAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: ClearRecoveryFlagAction(name, conf)
{
}

BT::NodeStatus ClearRecoveryFlagAction::tick()
{
  // 退出恢复模式
  setOutput("need_recovery", false);

  // 清理恢复流程相关计时器状态（避免下次复活沿/回血逻辑复用到旧值）
  setOutput("heal_start_ms", static_cast<std::uint64_t>(0));
  setOutput("search_start_ms", static_cast<std::uint64_t>(0));
  setOutput("recovery_start_ms", static_cast<std::uint64_t>(0));

  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::ClearRecoveryFlagAction, "ClearRecoveryFlag");
