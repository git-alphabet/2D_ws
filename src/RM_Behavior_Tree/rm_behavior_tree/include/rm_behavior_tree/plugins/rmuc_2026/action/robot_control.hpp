#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__ROBOT_CONTROL_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__ROBOT_CONTROL_HPP_

#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include "rm_decision_interfaces/msg/rmuc_robot_control.hpp"

namespace rm_behavior_tree
{

class RmucRobotControlAction : public BT::RosTopicPubNode<rm_decision_interfaces::msg::RMUCRobotControl>
{
public:
  RmucRobotControlAction(
    const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params);

  bool setMessage(rm_decision_interfaces::msg::RMUCRobotControl & msg) override;

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<bool>("stop_gimbal_scan"), BT::InputPort<bool>("chassis_spin"),
            BT::InputPort<bool>("fire_enable")};
  }
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__ROBOT_CONTROL_HPP_
