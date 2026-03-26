#ifndef RM_BEHAVIOR_TREE__PLUGINS__CONDITION__NOT_ARRIVED_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__CONDITION__NOT_ARRIVED_HPP_

#include <string>

#include "behaviortree_cpp/action_node.h"
#include "sp_msgs/msg/rmul.hpp"

namespace rm_behavior_tree
{

/**
 * @brief 未到达（未刷到补给区交互）判定节点（Condition-like）
 *
 * 黑板输入：
 *   - rfid_status: rm_decision_interfaces::msg::RFID
 *
 * 语义：
 *   - arrived := rfid_status.rfid_supply_arrived
 *   - arrived == false -> SUCCESS（NotArrived 成立：尚未刷到补给区交互）
 *   - arrived == true  -> FAILURE（已刷到补给区交互）
 *   - 若读不到 rfid_status -> FAILURE（保守失败）
 *
 * 设计说明：
 *  - 读不到输入时选择 FAILURE，避免“订阅链路断/黑板未更新”导致误判为未到达，
 *    进而触发错误的运动行为。
 */
class NotArrived : public BT::SyncActionNode
{
public:
  NotArrived(const std::string & name, const BT::NodeConfig & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<sp_msgs::msg::RMUL>("rfid_status")
    };
  }

  BT::NodeStatus tick() override;
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__CONDITION__NOT_ARRIVED_HPP_
