#include "rm_behavior_tree/plugins/rmuc_2026/action/sub_radar_tracks.hpp"

namespace rm_behavior_tree
{

RmucSubRadarTracksAction::RmucSubRadarTracksAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: BT::RosTopicSubNode<sp_msgs::msg::RMUCEnemyTracks>(name, conf, params)
{
}

BT::NodeStatus RmucSubRadarTracksAction::onTick(
  const std::shared_ptr<sp_msgs::msg::RMUCEnemyTracks> & last_msg)
{
  if (last_msg) {
    RCLCPP_DEBUG(logger(), "[%s] enemy_count=%d", name().c_str(), last_msg->enemy_count);
    setOutput("radar_tracks", *last_msg);
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::RmucSubRadarTracksAction, "SubRadarTracks");
