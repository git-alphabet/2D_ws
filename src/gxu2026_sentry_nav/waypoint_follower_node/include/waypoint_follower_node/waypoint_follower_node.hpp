// Waypoint collector and follower for foxglove
// Subscribes to /clicked_point, collects waypoints, sends to /follow_waypoints

#ifndef WAYPOINT_FOLLOWER_NODE__WAYPOINT_FOLLOWER_NODE_HPP_
#define WAYPOINT_FOLLOWER_NODE__WAYPOINT_FOLLOWER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>
#include <action_msgs/msg/goal_status.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <vector>

namespace waypoint_follower_node
{

class WaypointFollowerNode : public rclcpp::Node
{
public:
  using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
  using GoalHandle = rclcpp_action::ClientGoalHandle<FollowWaypoints>;

  explicit WaypointFollowerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void clicked_point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  bool start_callback();
  void clear_callback();
  void publish_markers();
  bool send_goal();

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_sub_;

  // Services
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_srv_;

  // Publisher
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  // Action client
  rclcpp_action::Client<FollowWaypoints>::SharedPtr action_client_;

  // State
  std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
  GoalHandle::SharedPtr goal_handle_;
  bool goal_pending_ = false;
};

}  // namespace waypoint_follower_node

#endif  // WAYPOINT_FOLLOWER_NODE__WAYPOINT_FOLLOWER_NODE_HPP_
