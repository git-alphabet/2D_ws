#ifndef RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_HP_INCREASING_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_HP_INCREASING_HPP_

#include "behaviortree_cpp/condition_node.h"
#include "rm_decision_interfaces/msg/rmul.hpp"

namespace rm_behavior_tree
{

/**
 * 检测HP是否在回升。
 *
 * 通过比较当前HP与上一次tick的HP判断。
 * 连续2次HP增加才返回SUCCESS，避免裁判系统消息抖动误判。
 *
 * 返回：SUCCESS=HP正在回升  FAILURE=HP未增加
 */
class IsHPIncreasingCondition : public BT::ConditionNode
{
public:
  IsHPIncreasingCondition(const std::string & name, const BT::NodeConfig & conf)
  : BT::ConditionNode(name, conf)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::shared_ptr<rm_decision_interfaces::msg::RMUL>>("message", "robot_status message"),
    };
  }

  BT::NodeStatus tick() override;

private:
  int last_hp_ = -1;
  int increase_count_ = 0;
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_HP_INCREASING_HPP_
