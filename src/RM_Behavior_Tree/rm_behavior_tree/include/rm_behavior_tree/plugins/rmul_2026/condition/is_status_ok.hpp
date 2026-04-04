#ifndef RM_BEHAVIOR_TREE__CONDITION__IS_STATUS_OK_HPP_
#define RM_BEHAVIOR_TREE__CONDITION__IS_STATUS_OK_HPP_

#include <string>
#include <memory>
#include "behaviortree_cpp/condition_node.h"
#include "sp_msgs/msg/rmul.hpp" // 统一使用 RMUL 消息

namespace rm_behavior_tree
{

class IsStatusOKAction : public BT::ConditionNode
{
public:
  IsStatusOKAction(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts()
  {
    // 定义输入端口及其默认值
    return {
      BT::InputPort<std::shared_ptr<sp_msgs::msg::RMUL>>("message", "Robot status message"),
      BT::InputPort<int>("hp_threshold", 0, "Minimum acceptable robot HP (inclusive)"),
      BT::InputPort<int>("heat_threshold", 9999, "Maximum acceptable shooter heat (inclusive)")
    };
  }

  BT::NodeStatus tick() override;

private:
  // 注意：SimpleConditionNode 的构造函数需要一个 tick 函数指针，
  // 但直接使用 tick() 更符合 BT::ConditionNode 的标准用法。
  // 我们将覆盖 tick() 而不是使用 SimpleConditionNode 的绑定方式。
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__CONDITION__IS_STATUS_OK_HPP_
