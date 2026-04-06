#include "rm_behavior_tree/plugins/rmul_2026/condition/is_status_ok.hpp"

namespace rm_behavior_tree
{

IsStatusOKAction::IsStatusOKAction(const std::string & name, const BT::NodeConfig & config)
: BT::ConditionNode(name, config)
{
}

BT::NodeStatus IsStatusOKAction::tick()
{
  // 获取输入参数
  auto msg = getInput<std::shared_ptr<sp_msgs::msg::RMUL>>("message");
  int hp_threshold = 0;
  int heat_threshold = 9999;

  // 获取阈值，没有设置则使用默认值
  getInput("hp_threshold", hp_threshold);
  getInput("heat_threshold", heat_threshold);

  // 检查消息是否有效
  if (!msg) {
    return BT::NodeStatus::FAILURE;
  }

  // 检查血量和热量条件
  bool hp_ok = (*msg)->current_hp >= hp_threshold;
  bool heat_ok = (*msg)->shooter_heat <= heat_threshold;

  if (hp_ok && heat_ok) {
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

}  // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::IsStatusOKAction>("IsStatusOK");
}
