#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__CLEAR_RECOVERY_FLAG_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__CLEAR_RECOVERY_FLAG_HPP_

#include <cstdint>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"

namespace rm_behavior_tree
{

class ClearRecoveryFlagAction : public BT::SyncActionNode
{
public:
  ClearRecoveryFlagAction(const std::string & name, const BT::NodeConfig & conf);
  ClearRecoveryFlagAction(const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts()
  {
    return {
      BT::BidirectionalPort<bool>("need_recovery"),
      BT::BidirectionalPort<std::uint64_t>("heal_start_ms"),
      BT::BidirectionalPort<std::uint64_t>("search_start_ms"),
      BT::BidirectionalPort<std::uint64_t>("recovery_start_ms")
    };
  }

  BT::NodeStatus tick() override;
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__CLEAR_RECOVERY_FLAG_HPP_
