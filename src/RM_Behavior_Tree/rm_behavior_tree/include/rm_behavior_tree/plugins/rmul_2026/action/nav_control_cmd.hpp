#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__NAV_CONTROL_CMD_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__NAV_CONTROL_CMD_HPP_

#include <string>

#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include "sp_msgs/msg/rmul.hpp"

namespace rm_behavior_tree
{

class NavControlCmdAction : public BT::RosTopicPubNode<sp_msgs::msg::RMUL>
{
public:
  NavControlCmdAction(
    const std::string & name,
    const BT::NodeConfig & conf,
    const BT::RosNodeParams & params);

  bool setMessage(sp_msgs::msg::RMUL & msg) override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("cmd_type"),
      BT::InputPort<bool>("emergency_stop")
    };
  }
}; 

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__NAV_CONTROL_CMD_HPP_
