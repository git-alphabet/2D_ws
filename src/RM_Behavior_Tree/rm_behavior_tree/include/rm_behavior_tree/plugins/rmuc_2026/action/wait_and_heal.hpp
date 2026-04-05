#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__WAIT_AND_HEAL_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__WAIT_AND_HEAL_HPP_

#include <cstdint>
#include <string>

#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "sp_msgs/msg/rmuc_robot_status.hpp"

namespace rm_behavior_tree
{

class RmucWaitAndHealAction : public BT::RosTopicSubNode<sp_msgs::msg::RMUCRobotStatus>
{
public:
  RmucWaitAndHealAction(
    const std::string & name,
    const BT::NodeConfig & conf,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic_name"),
      BT::InputPort<std::uint64_t>("now_ms"),
      BT::InputPort<int>("hp_cur"),
      BT::InputPort<int>("hp_max"),
      BT::BidirectionalPort<std::uint64_t>("heal_start_ms"),
      BT::InputPort<std::uint64_t>("heal_wait_ms"),
      BT::InputPort<double>("heal_min_ratio")
    };
  }

  BT::NodeStatus onTick(
    const std::shared_ptr<sp_msgs::msg::RMUCRobotStatus> & last_msg) override;

private:
  static constexpr int MAX_HP_FIXED = 400;
  bool has_hp_cache_ = false;
  int last_hp_cache_ = 0;
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__WAIT_AND_HEAL_HPP_
