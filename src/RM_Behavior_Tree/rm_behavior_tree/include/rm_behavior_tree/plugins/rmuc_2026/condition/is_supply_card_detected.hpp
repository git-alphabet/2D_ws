#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__CONDITION__IS_SUPPLY_CARD_DETECTED_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__CONDITION__IS_SUPPLY_CARD_DETECTED_HPP_

#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "sp_msgs/msg/rmucrfid_status.hpp"

namespace rm_behavior_tree
{

class RmucIsSupplyCardDetectedCondition : public BT::ConditionNode
{
public:
  RmucIsSupplyCardDetectedCondition(
    const std::string & name,
    const BT::NodeConfig & conf,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<sp_msgs::msg::RMUCRFIDStatus>("rfid_status")
    };
  }

  BT::NodeStatus tick() override;

private:
  BT::RosNodeParams params_;
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__CONDITION__IS_SUPPLY_CARD_DETECTED_HPP_
