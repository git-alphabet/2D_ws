#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__IS_FRIEND_OK_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__IS_FRIEND_OK_HPP_

#include "behaviortree_cpp/condition_node.h"
#include "rm_decision_interfaces/msg/rmul.hpp"

namespace rm_behavior_tree
{

/**
 * @brief condition节点，用于判断队友状态
 *
 * 注意：由于 RMUL.msg 中不再包含 AllRobotHP 信息，
 * 此节点已降级为始终返回 SUCCESS。
 * 如需恢复队友血量对比功能，请在 RMUL.msg 中重新添加队友 HP 字段。
 */
class IsFriendOKAction : public BT::SimpleConditionNode
{
public:
  IsFriendOKAction(const std::string & name, const BT::NodeConfig & config);

  BT::NodeStatus checkFriendStatus();

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<rm_decision_interfaces::msg::RMUL>("message")};
  }
};
}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__IS_FRIEND_OK_HPP_