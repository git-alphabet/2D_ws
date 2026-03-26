#ifndef RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_HP_BELOW_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_HP_BELOW_HPP_

#include <string>
#include <memory>
#include "behaviortree_cpp/condition_node.h"
#include "sp_msgs/msg/rmul.hpp"

namespace rm_behavior_tree
{
/**

@brief Condition节点：判断当前血量是否小于阈值

从输入端口获取机器人状态消息和血量阈值。

若 current_hp < hp_threshold 返回 SUCCESS，否则返回 FAILURE。

@param[in] message 机器人状态消息（含 current_hp）

@param[in] hp_threshold 血量阈值
*/
class IsHPBelowCondition : public BT::SimpleConditionNode
{
public:
IsHPBelowCondition(const std::string & name, const BT::NodeConfig & config);

BT::NodeStatus checkHPBelow();

static BT::PortsList providedPorts()
{
return {
BT::InputPort<std::shared_ptr<sp_msgs::msg::RMUL>>("message"),
BT::InputPort<int>("hp_threshold")
};
}
};

} // namespace rm_behavior_tree

#endif // RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_HP_BELOW_HPP_