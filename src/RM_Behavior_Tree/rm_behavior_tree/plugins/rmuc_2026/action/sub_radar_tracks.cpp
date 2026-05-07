#include "rm_behavior_tree/plugins/rmuc_2026/action/sub_radar_tracks.hpp"

namespace rm_behavior_tree
{

SubRadarTracksAction::SubRadarTracksAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: BT::RosTopicSubNode<sp_msgs::msg::RMUCEnemyTracks>(name, conf, params)
{
}

BT::NodeStatus SubRadarTracksAction::onTick(
  const std::shared_ptr<sp_msgs::msg::RMUCEnemyTracks> & last_msg)
{
  if (last_msg) {
    RCLCPP_DEBUG(
      logger(), "[%s] radar_tracks: count=%u",
      name().c_str(), static_cast<unsigned>(last_msg->enemy_x.size()));
    setOutput("radar_tracks", *last_msg);
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::SubRadarTracksAction, "SubRadarTracks");
