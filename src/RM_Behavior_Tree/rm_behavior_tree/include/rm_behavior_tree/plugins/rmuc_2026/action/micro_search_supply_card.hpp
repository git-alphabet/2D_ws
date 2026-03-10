#ifndef RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__MICRO_SEARCH_SUPPLY_CARD_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__MICRO_SEARCH_SUPPLY_CARD_HPP_

#include <cstdint>
#include <string>
#include <memory>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "rm_decision_interfaces/msg/rmucrfid_status.hpp"

namespace rm_behavior_tree
{

class RmucMicroSearchSupplyCardAction : public BT::StatefulActionNode
{
public:
  RmucMicroSearchSupplyCardAction(
    const std::string& name,
    const BT::NodeConfig& config,
    const BT::RosNodeParams& params);

  static BT::PortsList providedPorts()
  {
    return {
      BT::BidirectionalPort<std::uint64_t>("search_start_ms"),
      BT::InputPort<std::uint64_t>("timeout_ms"),
      BT::InputPort<rm_decision_interfaces::msg::RMUCRFIDStatus>("rfid_status"),
      BT::InputPort<bool>("rfid_supply_arrived")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::uint64_t nowMs_() const;
  bool getCurrentPose_(geometry_msgs::msg::PoseStamped& out_pose);
  void publishNextGoal_(const geometry_msgs::msg::PoseStamped& cur);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_goal_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string map_frame_{"map"};
  std::string base_frame_{"chassis"};

  int64_t last_pub_ms_{0};
  size_t step_idx_{0};
  int ring_idx_{0};
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__RMUC_2026__ACTION__MICRO_SEARCH_SUPPLY_CARD_HPP_
