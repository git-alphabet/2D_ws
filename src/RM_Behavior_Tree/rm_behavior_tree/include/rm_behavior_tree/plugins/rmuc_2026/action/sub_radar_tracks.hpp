#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__SUB_RADAR_TRACKS_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__SUB_RADAR_TRACKS_HPP_

#include <string>
#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "rm_decision_interfaces/msg/rmuc_enemy_tracks.hpp"

namespace rm_behavior_tree
{
/// 订阅 RMUC 消息中的雷达敌方目标字段，将整条消息写入黑板 {radar.tracks}
class RmucSubRadarTracksAction : public BT::RosTopicSubNode<rm_decision_interfaces::msg::RMUCEnemyTracks>
{
public:
  RmucSubRadarTracksAction(
    const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic_name"),
      BT::OutputPort<rm_decision_interfaces::msg::RMUCEnemyTracks>("radar_tracks")};
  }

  BT::NodeStatus onTick(
    const std::shared_ptr<rm_decision_interfaces::msg::RMUCEnemyTracks> & last_msg) override;
};
}  // namespace rm_behavior_tree

#endif
