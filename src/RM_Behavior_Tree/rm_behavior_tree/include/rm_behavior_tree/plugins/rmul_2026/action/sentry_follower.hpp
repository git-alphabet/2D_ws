// sentry_follower.hpp
#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__SENTRY_FOLLOWER_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__SENTRY_FOLLOWER_HPP_

#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <mutex>
#include <atomic>

namespace rm_behavior_tree
{

class SentryFollower : public BT::RosTopicPubNode<geometry_msgs::msg::PoseStamped>
{
public:
  SentryFollower(const std::string & name,
                const BT::NodeConfiguration & config,
                const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();
  bool setMessage(geometry_msgs::msg::PoseStamped & msg) override;

private:
  void armorPositionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  
  std::mutex data_mutex_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr armor_sub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  geometry_msgs::msg::PoseStamped current_armor_;
  std::atomic<bool> new_data_{false};
  rclcpp::Time last_valid_time_;
};

} // namespace rm_behavior_tree

#endif // RM_BEHAVIOR_TREE__PLUGINS__ACTION__SENTRY_FOLLOWER_HPP_

