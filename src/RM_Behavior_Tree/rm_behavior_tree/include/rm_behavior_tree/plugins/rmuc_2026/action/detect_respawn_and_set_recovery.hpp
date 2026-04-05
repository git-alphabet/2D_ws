#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__DETECT_RESPAWN_AND_SET_RECOVERY_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__DETECT_RESPAWN_AND_SET_RECOVERY_HPP_

#include <cstdint>
#include <string>
#include <algorithm>
#include <limits>

#include <boost/signals2/connection.hpp>
#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "sp_msgs/msg/rmuc_robot_status.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rm_behavior_tree
{

class RmucDetectRespawnAndSetRecoveryAction
: public BT::RosTopicSubNode<sp_msgs::msg::RMUCRobotStatus>
{
public:
  RmucDetectRespawnAndSetRecoveryAction(
    const std::string & name,
    const BT::NodeConfig & conf,
    const BT::RosNodeParams & params);

  ~RmucDetectRespawnAndSetRecoveryAction() override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::BidirectionalPort<int>("hp_cur"),
      BT::BidirectionalPort<bool>("was_dead"),
      BT::BidirectionalPort<bool>("need_recovery"),
      BT::BidirectionalPort<std::uint64_t>("recovery_start_ms"),
      BT::BidirectionalPort<std::uint64_t>("search_start_ms"),
      BT::BidirectionalPort<std::uint64_t>("heal_start_ms")
    });
  }

  BT::NodeStatus onTick(
    const std::shared_ptr<sp_msgs::msg::RMUCRobotStatus> & last_msg) override;

private:
  static constexpr int RESPAWN_STABLE_FRAMES = 2;
  int alive_stable_frames_ = 0;
  bool respawn_locked_ = false;
  static constexpr std::uint64_t RESPAWN_LOCK_TIMEOUT = 5000;
  std::uint64_t last_respawn_ms_ = 0;
  int current_hp_ = 0;
  rclcpp::Subscription<sp_msgs::msg::RMUCRobotStatus>::SharedPtr fallback_sub_;
  std::shared_ptr<sp_msgs::msg::RMUCRobotStatus> fallback_last_msg_ = nullptr;
  boost::signals2::connection fallback_signal_conn_;
  static constexpr int MAX_HP = 400;
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__DETECT_RESPAWN_AND_SET_RECOVERY_HPP_
