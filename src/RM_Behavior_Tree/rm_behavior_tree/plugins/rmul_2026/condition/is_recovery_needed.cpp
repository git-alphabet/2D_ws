#include "rm_behavior_tree/plugins/rmul_2026/condition/is_recovery_needed.hpp"

#include "behaviortree_cpp/exceptions.h"

namespace rm_behavior_tree
{

IsRecoveryNeededCondition::IsRecoveryNeededCondition(
  const std::string & name,
  const BT::NodeConfig & conf,
  const BT::RosNodeParams & params)
: BT::ConditionNode(name, conf), params_(params)
{
}

BT::NodeStatus IsRecoveryNeededCondition::tick()
{
  // 读黑板 need_recovery
  auto res = getInput<bool>("need_recovery");
  if (!res) {
    // 首次 tick 时黑板中可能还没有 need_recovery 键
    // （DetectRespawnAndSetRecovery 尚未收到消息时会跳过写入）
    // 安全降级：视为不需要恢复
    RCLCPP_DEBUG(rclcpp::get_logger("IsRecoveryNeeded"),
      "need_recovery key not found in blackboard, default to FAILURE (no recovery)");
    return BT::NodeStatus::FAILURE;
  }

  return (res.value() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE);
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::IsRecoveryNeededCondition, "IsRecoveryNeeded");
