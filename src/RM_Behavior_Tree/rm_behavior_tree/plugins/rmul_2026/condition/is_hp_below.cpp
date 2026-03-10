#include "rm_behavior_tree/plugins/rmul_2026/condition/is_hp_below.hpp"

#include <functional>

namespace rm_behavior_tree
{

IsHPBelowCondition::IsHPBelowCondition(const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsHPBelowCondition::checkHPBelow, this), config)
{
}

BT::NodeStatus IsHPBelowCondition::checkHPBelow()
{
int hp_threshold = 0;

auto msg = getInput<std::shared_ptr<rm_decision_interfaces::msg::RMUL>>("message");
if (!getInput("hp_threshold", hp_threshold)) {
// 没有传阈值就用默认 0
hp_threshold = 0;
}

if (!msg) {
// 缺消息：保守失败
return BT::NodeStatus::FAILURE;
}

if ((*msg)->current_hp < hp_threshold) {
return BT::NodeStatus::SUCCESS;
} else {
return BT::NodeStatus::FAILURE;
}
}

} // namespace rm_behavior_tree

#include "behaviortree_cpp/bt_factory.h"

BT_REGISTER_NODES(factory)
{
factory.registerNodeType<rm_behavior_tree::IsHPBelowCondition>("IsHPBelow");
}