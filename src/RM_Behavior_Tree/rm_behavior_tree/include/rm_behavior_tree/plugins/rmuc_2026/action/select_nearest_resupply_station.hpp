#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__SELECT_NEAREST_RESUPPLY_STATION_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__SELECT_NEAREST_RESUPPLY_STATION_HPP_

#include <string>
#include <cmath>
#include "behaviortree_cpp/action_node.h"

namespace rm_behavior_tree
{
/// 从补给区/基地增益区/前哨增益区中选择距自身最近的补给点
class SelectNearestResupplyStationAction : public BT::SyncActionNode
{
public:
  SelectNearestResupplyStationAction(const std::string & name, const BT::NodeConfig & conf);
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("pose_x"), BT::InputPort<double>("pose_y"),
      BT::InputPort<double>("supply_zone_x"), BT::InputPort<double>("supply_zone_y"),
      BT::InputPort<double>("base_buff_x"), BT::InputPort<double>("base_buff_y"),
      BT::InputPort<double>("outpost_buff_x"), BT::InputPort<double>("outpost_buff_y"),
      BT::InputPort<bool>("outpost_alive", "true", "outpost_alive"),
      BT::OutputPort<double>("goal_x"), BT::OutputPort<double>("goal_y")};
  }
  BT::NodeStatus tick() override;
};
}  // namespace rm_behavior_tree
#endif
