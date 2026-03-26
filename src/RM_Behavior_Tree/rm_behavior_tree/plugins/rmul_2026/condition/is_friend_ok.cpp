#include "rm_behavior_tree/plugins/rmul_2026/condition/is_friend_ok.hpp"

namespace rm_behavior_tree
{

IsFriendOKAction::IsFriendOKAction(const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsFriendOKAction::checkFriendStatus, this), config)
{
}

BT::NodeStatus IsFriendOKAction::checkFriendStatus()
{
  // RMUL.msg 不再包含 AllRobotHP 信息，无法对比友旌血量。
  // 降级处理：始终返回 SUCCESS。
  // TODO: 如需恢复队友血量对比，请在 RMUL.msg 中添加相关字段。
  (void)getInput<sp_msgs::msg::RMUL>("message");
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::IsFriendOKAction>("IsFriendOK");
}
