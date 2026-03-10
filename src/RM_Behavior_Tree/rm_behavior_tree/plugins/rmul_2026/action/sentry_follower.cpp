// sentry_follower.cpp
#include "rm_behavior_tree/plugins/rmul_2026/action/sentry_follower.hpp"

namespace rm_behavior_tree
{

SentryFollower::SentryFollower(
  const std::string & name,
  const BT::NodeConfiguration & config,
  const BT::RosNodeParams & params)
: BT::RosTopicPubNode<geometry_msgs::msg::PoseStamped>(name, config, params),
  last_valid_time_(node_->now())
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  armor_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/armor_position", rclcpp::SensorDataQoS(),
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      try {
        geometry_msgs::msg::PoseStamped transformed;
        tf_buffer_->transform(*msg, transformed, "map", 
                             tf2::durationFromSec(0.2));

        // 数据有效性检查
        if(std::isnan(transformed.pose.position.x) || 
           std::isnan(transformed.pose.position.y)) {
          throw tf2::TransformException("Invalid transformed position");
        }

        std::lock_guard<std::mutex> lock(data_mutex_);
        current_armor_ = transformed;
        new_data_.store(true, std::memory_order_release);
        last_valid_time_ = node_->now();
        
        RCLCPP_DEBUG(node_->get_logger(), "New valid armor data received");
        this->emitWakeUpSignal();
      } 
      catch (const tf2::TransformException &ex) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "Transform error: %s", ex.what());
      }
    });
}

BT::PortsList SentryFollower::providedPorts()
{
  return {
    BT::InputPort<std::string>("topic", "/goal_pose"),
    BT::InputPort<double>("follow_distance", 2.0, "Following distance (meters)"),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>("current_goal", "Current target position")
  };
}

bool SentryFollower::setMessage(geometry_msgs::msg::PoseStamped & msg)
{
  // 检查数据新鲜度（2秒超时）
  if ((node_->now() - last_valid_time_).seconds() > 2.0) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                        "Armor data expired (%.1fs old)", 
                        (node_->now() - last_valid_time_).seconds());
    return false;
  }

  // 原子操作检查新数据标志
  if (!new_data_.exchange(false, std::memory_order_acq_rel)) {
    RCLCPP_DEBUG(node_->get_logger(), "No new data since last processing");
    return false;
  }

  double follow_distance;
  if (!getInput("follow_distance", follow_distance)) {
    RCLCPP_ERROR(node_->get_logger(), "Missing follow_distance parameter");
    return false;
  }

  auto current_pose = config().blackboard->get<geometry_msgs::msg::PoseStamped>("current_pose");

  geometry_msgs::msg::PoseStamped local_armor;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    local_armor = current_armor_;
  }

  const double dx = local_armor.pose.position.x - current_pose.pose.position.x;
  const double dy = local_armor.pose.position.y - current_pose.pose.position.y;
  const double distance = std::hypot(dx, dy);

  if (distance > 0.01) {
    const double ratio = follow_distance / distance;
    msg.pose.position.x = current_pose.pose.position.x + dx * ratio;
    msg.pose.position.y = current_pose.pose.position.y + dy * ratio;

    tf2::Quaternion q;
    q.setRPY(0, 0, std::atan2(dy, dx));
    msg.pose.orientation = tf2::toMsg(q.normalized());
  } else {
    msg.pose = current_pose.pose;
  }

  msg.header.stamp = node_->now();
  msg.header.frame_id = "map";
  setOutput("current_goal", msg);

  return true;
}

} // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::SentryFollower, "SentryFollower")