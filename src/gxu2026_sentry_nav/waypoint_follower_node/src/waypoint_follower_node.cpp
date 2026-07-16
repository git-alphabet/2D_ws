// Waypoint collector and follower for foxglove
// Click map to collect, call /start_waypoints to follow, /clear_waypoints to reset

#include "waypoint_follower_node/waypoint_follower_node.hpp"

using namespace std::placeholders;

namespace waypoint_follower_node
{

WaypointFollowerNode::WaypointFollowerNode(const rclcpp::NodeOptions & options)
: Node("waypoint_follower_node", options)
{
  // Subscribe to foxglove's "Publish Point" topic
  clicked_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/clicked_point", 10,
    std::bind(&WaypointFollowerNode::clicked_point_callback, this, _1));

  // Services: call from foxglove service panel
  start_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "/start_waypoints",
    [this](const std_srvs::srv::Trigger::Request::SharedPtr,
           std_srvs::srv::Trigger::Response::SharedPtr res) {
      res->success = this->start_callback();
      res->message = res->success
        ? "Sent " + std::to_string(waypoints_.size()) + " waypoints"
        : "Failed to send waypoints";
    });

  clear_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "/clear_waypoints",
    [this](const std_srvs::srv::Trigger::Request::SharedPtr,
           std_srvs::srv::Trigger::Response::SharedPtr res) {
      this->clear_callback();
      res->success = true;
      res->message = "Waypoints cleared";
    });

  // Publisher for visualization
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/waypoint_markers", 10);

  // Action client for follow_waypoints
  action_client_ = rclcpp_action::create_client<FollowWaypoints>(
    this, "/follow_waypoints");

  RCLCPP_INFO(this->get_logger(), "Waypoint Follower Node started");
  RCLCPP_INFO(this->get_logger(), "  Click map in foxglove to add waypoints");
  RCLCPP_INFO(this->get_logger(), "  Call /start_waypoints to follow");
  RCLCPP_INFO(this->get_logger(), "  Call /clear_waypoints to reset");
}

void WaypointFollowerNode::clicked_point_callback(
  const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header = msg->header;
  pose.pose.position = msg->point;
  pose.pose.orientation.w = 1.0;
  waypoints_.push_back(pose);

  RCLCPP_INFO(this->get_logger(), "Waypoint %zu: (%.2f, %.2f)",
    waypoints_.size(), msg->point.x, msg->point.y);

  publish_markers();
}

bool WaypointFollowerNode::start_callback()
{
  if (waypoints_.empty()) {
    RCLCPP_WARN(this->get_logger(), "No waypoints collected");
    return false;
  }

  return send_goal();
}

void WaypointFollowerNode::clear_callback()
{
  // Cancel active goal if any
  auto status = goal_handle_ ? goal_handle_->get_status() : 0;
  if (goal_handle_ &&
      (status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
       status == action_msgs::msg::GoalStatus::STATUS_EXECUTING)) {
    action_client_->async_cancel_goal(goal_handle_);
    RCLCPP_INFO(this->get_logger(), "Canceled active goal");
  }
  goal_handle_.reset();
  goal_pending_ = false;
  waypoints_.clear();
  publish_markers();
  RCLCPP_INFO(this->get_logger(), "Waypoints cleared");
}

void WaypointFollowerNode::publish_markers()
{
  visualization_msgs::msg::MarkerArray array;

  // Delete old markers
  visualization_msgs::msg::Marker clear;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  array.markers.push_back(clear);

  // Add waypoint markers
  for (size_t i = 0; i < waypoints_.size(); ++i) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = waypoints_[i].header.frame_id;
    marker.header.stamp = this->now();
    marker.ns = "waypoints";
    marker.id = static_cast<int>(i);
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = waypoints_[i].pose;
    marker.scale.x = 0.15;
    marker.scale.y = 0.15;
    marker.scale.z = 0.15;
    marker.color.r = 1.0;
    marker.color.g = 0.5;
    marker.color.b = 0.0;
    marker.color.a = 0.9;
    array.markers.push_back(marker);

    // Number label
    visualization_msgs::msg::Marker text;
    text.header = marker.header;
    text.ns = "waypoint_numbers";
    text.id = static_cast<int>(i);
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;
    text.pose = waypoints_[i].pose;
    text.pose.position.z += 0.2;
    text.scale.z = 0.15;
    text.color.r = 1.0;
    text.color.g = 1.0;
    text.color.b = 1.0;
    text.color.a = 0.9;
    text.text = std::to_string(i + 1);
    array.markers.push_back(text);
  }

  marker_pub_->publish(array);
}

bool WaypointFollowerNode::send_goal()
{
  // Guard: reject if a goal is already active or pending
  auto st = goal_handle_ ? goal_handle_->get_status() : 0;
  if (goal_pending_ ||
      (goal_handle_ &&
       (st == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
        st == action_msgs::msg::GoalStatus::STATUS_EXECUTING))) {
    RCLCPP_WARN(this->get_logger(), "Goal already active, ignoring");
    return false;
  }

  // Non-blocking check for action server
  if (!action_client_->action_server_is_ready()) {
    RCLCPP_ERROR(this->get_logger(), "follow_waypoints action server not available");
    return false;
  }

  goal_pending_ = true;

  auto goal = FollowWaypoints::Goal();
  goal.poses = waypoints_;

  RCLCPP_INFO(this->get_logger(), "Sending %zu waypoints...", waypoints_.size());

  auto send_goal_options = rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    [this](const GoalHandle::SharedPtr & handle) {
      if (!handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal rejected by server");
        goal_pending_ = false;
        return;
      }
      goal_handle_ = handle;
      goal_pending_ = false;
      RCLCPP_INFO(this->get_logger(), "Goal accepted");
    };
  send_goal_options.result_callback =
    [this](const GoalHandle::WrappedResult & result) {
      goal_handle_.reset();
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(this->get_logger(), "All waypoints reached");
      } else {
        RCLCPP_WARN(this->get_logger(), "Waypoint following failed or was canceled");
      }
    };

  try {
    action_client_->async_send_goal(goal, send_goal_options);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to send goal: %s", e.what());
    goal_pending_ = false;
    return false;
  }
  return true;
}

}  // namespace waypoint_follower_node

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<waypoint_follower_node::WaypointFollowerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
