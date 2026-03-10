#ifndef RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_AT_NAV_GOAL_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_AT_NAV_GOAL_HPP_

#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "rm_decision_interfaces/msg/rmul.hpp"

namespace rm_behavior_tree
{

/**
 * @brief 判断机器人是否到达导航目标点
 *
 * 黑板输入：
 *   - rfid_status: rm_decision_interfaces::msg::RMUL
 *
 * 语义：
 *   - is_at_nav_goal == true  -> SUCCESS（已到达目标点）
 *   - is_at_nav_goal == false -> FAILURE（未到达目标点）
 *   - 若读不到 rfid_status -> FAILURE（保守失败）
 *
 * 设计说明：
 *  - 基于 RMUL.msg 第9个字段 is_at_nav_goal 进行判断
 *  - 读不到输入时选择 FAILURE，避免订阅链路断线导致误判
 */
class IsAtNavGoal : public BT::ConditionNode
{
public:
  IsAtNavGoal(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<rm_decision_interfaces::msg::RMUL>("rfid_status")
    };
  }

  BT::NodeStatus tick() override;
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_AT_NAV_GOAL_HPP_
