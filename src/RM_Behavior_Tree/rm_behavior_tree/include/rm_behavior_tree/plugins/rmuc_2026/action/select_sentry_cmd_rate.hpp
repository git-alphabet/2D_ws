#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__SELECT_SENTRY_CMD_RATE_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__SELECT_SENTRY_CMD_RATE_HPP_

#include <string>

#include "behaviortree_cpp/action_node.h"

namespace rm_behavior_tree
{
class SelectSentryCmdRateAction : public BT::SyncActionNode
{
public:
  SelectSentryCmdRateAction(const std::string & name, const BT::NodeConfig & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<bool>("is_dead"),
      BT::InputPort<double>("alive_hz", 0.2, "alive sentry_cmd publish hz"),
      BT::InputPort<double>("dead_hz", 1.0, "dead sentry_cmd publish hz"),
      BT::OutputPort<double>("hz_out")};
  }

  BT::NodeStatus tick() override;
};
}  // namespace rm_behavior_tree

#endif