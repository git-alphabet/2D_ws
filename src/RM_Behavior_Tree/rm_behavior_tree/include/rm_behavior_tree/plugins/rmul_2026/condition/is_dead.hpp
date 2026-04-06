#ifndef RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_DEAD_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_DEAD_HPP_

#include <string>
#include <memory>
#include "behaviortree_cpp/condition_node.h"
#include "sp_msgs/msg/rmul.hpp"

namespace rm_behavior_tree
{
/**
 * @brief Condition节点：判断当前血量是否为0（是否死亡）
 *
 * 从输入端口获取机器人状态消息，判断 current_hp 是否等于 0。
 * 若 current_hp == 0 返回 SUCCESS，否则返回 FAILURE。
 *
 * @param[in] message  机器人状态消息（含 current_hp）
 */
class IsDeadCondition : public BT::SimpleConditionNode
{
public:
  IsDeadCondition(const std::string & name, const BT::NodeConfig & config);

  BT::NodeStatus checkDead();

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::shared_ptr<sp_msgs::msg::RMUL>>("message")
    };
  }
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_DEAD_HPP_
