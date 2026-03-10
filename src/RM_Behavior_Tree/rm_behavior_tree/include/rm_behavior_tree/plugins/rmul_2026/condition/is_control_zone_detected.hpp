#ifndef RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_CONTROL_ZONE_DETECTED_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_CONTROL_ZONE_DETECTED_HPP_

#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "rm_decision_interfaces/msg/rmul.hpp"

namespace rm_behavior_tree
{

/**
 * @brief Condition节点：判断是否成功与控制区产生交互
 * 
 * 从输入端口获取RFID状态消息，检查 rfid_control_arrived 字段。
 * 若成功与控制区交互 (rfid_control_arrived == true) 返回 SUCCESS，否则返回 FAILURE。
 * 
 * @param[in] rfid_status RFID状态消息（RMUL类型，包含 rfid_control_arrived 字段）
 */
class IsControlZoneDetectedCondition : public BT::ConditionNode
{
public:
  IsControlZoneDetectedCondition(
    const std::string & name,
    const BT::NodeConfig & conf,
    const BT::RosNodeParams & params);
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<rm_decision_interfaces::msg::RMUL>("rfid_status")
    };
  }
  BT::NodeStatus tick() override;

private:
  // 保留 params_ 与其它 ROS 节点构造签名一致
  BT::RosNodeParams params_;
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_CONTROL_ZONE_DETECTED_HPP_
