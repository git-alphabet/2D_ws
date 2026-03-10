#include "rm_behavior_tree/plugins/rmul_2026/condition/is_detect_enemy.hpp"

namespace rm_behavior_tree
{

IsDetectEnemyCondition::IsDetectEnemyCondition(const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsDetectEnemyCondition::checkEnemyDetected, this), config)
{
}

BT::NodeStatus IsDetectEnemyCondition::checkEnemyDetected()
{
  auto msg = getInput<std::shared_ptr<rm_decision_interfaces::msg::RMUL>>("message");

  if (!msg) {
    return BT::NodeStatus::FAILURE;
  }

  // 使用RMUL消息中的视觉检测敌人字段
  if ((*msg)->is_detect_enemy) {
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::IsDetectEnemyCondition>("IsDetectEnemy");
}
