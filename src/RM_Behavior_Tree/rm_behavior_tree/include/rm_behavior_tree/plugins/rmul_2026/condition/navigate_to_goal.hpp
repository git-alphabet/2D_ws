#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__NAVIGATE_TO_GOAL_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__NAVIGATE_TO_GOAL_HPP_

#include <string>

#include "behaviortree_cpp/action_node.h"
#include "rm_decision_interfaces/msg/rmul.hpp"

namespace rm_behavior_tree
{

/**
 * @brief 纯 RFID 条件判断节点（Condition-like）
 *
 * 语义：
 *   - 从黑板读取 RFID.msg（端口：rfid_status）
 *   - 若 rfid_supply_arrived == true  -> SUCCESS
 *   - 若 rfid_supply_arrived == false -> FAILURE
 *   - 若读不到 rfid_status            -> FAILURE（保守失败）
 *
 * 注意：
 *  - rfid_status 端口期望的是黑板中的 RFID.msg 对象（由订阅/上游节点写入）。
 *    不应在 XML 中以字符串方式配置该端口，否则可能触发 convertFromString 失败。
 *  - 本节点不写回黑板，不修改 RFID 状态，仅读取并判断。
 *  - BT 会周期性 tick 本节点，因此它会“持续判断当前黑板上的状态”。
 */
class NavigateToGoal : public BT::SyncActionNode
{
public:
  NavigateToGoal(const std::string & name, const BT::NodeConfig & conf);

  static BT::PortsList providedPorts()
  {
    return {
      // RFID 状态：由其他节点订阅 RFID.msg 并写入黑板后提供给此节点读取
      BT::InputPort<rm_decision_interfaces::msg::RMUL>("rfid_status")
    };
  }

  BT::NodeStatus tick() override;
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__NAVIGATE_TO_GOAL_HPP_
