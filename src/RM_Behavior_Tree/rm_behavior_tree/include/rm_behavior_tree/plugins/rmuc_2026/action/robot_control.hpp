#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__ROBOT_CONTROL_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__ROBOT_CONTROL_HPP_

#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include "sp_msgs/msg/rmuc_robot_control.hpp"

namespace rm_behavior_tree
{

class RmucRobotControlAction : public BT::RosTopicPubNode<sp_msgs::msg::RMUCRobotControl>
{
public:
  RmucRobotControlAction(
    const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params);

  bool setMessage(sp_msgs::msg::RMUCRobotControl & msg) override;

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<bool>("stop_gimbal_scan"), BT::InputPort<bool>("chassis_spin")};
  }
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__ROBOT_CONTROL_HPP_
