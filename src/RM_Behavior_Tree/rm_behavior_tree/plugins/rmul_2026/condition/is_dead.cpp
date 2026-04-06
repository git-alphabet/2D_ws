#include "rm_behavior_tree/plugins/rmul_2026/condition/is_dead.hpp"

#include <functional>

namespace rm_behavior_tree
{

IsDeadCondition::IsDeadCondition(const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsDeadCondition::checkDead, this), config)
{
}

BT::NodeStatus IsDeadCondition::checkDead()
{
  // 获取机器人状态消息输入
   auto msg = getInput<std::shared_ptr<sp_msgs::msg::RMUL>>("message");

  // 缺消息：保守返回失败（未获取到状态则无法判定死亡）
  if (!msg) {
    return BT::NodeStatus::FAILURE;
  }

   // 判断血量是否为0
   if ((*msg)->current_hp <= 0) {
    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::IsDeadCondition>("IsDead");
}
