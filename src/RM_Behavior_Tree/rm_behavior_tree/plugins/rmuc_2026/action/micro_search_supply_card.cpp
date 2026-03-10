#include "rm_behavior_tree/plugins/rmuc_2026/action/micro_search_supply_card.hpp"
#include "behaviortree_ros2/plugins.hpp"

#include <iostream>

// --------- TF lookup 的签名自适配工具 ---------
namespace
{
template <typename BufferT>
auto lookupTfCompat(BufferT & buf, const std::string & target, const std::string & source, int)
  -> decltype(buf.lookupTransform(target, source, tf2::TimePointZero))
{
  return buf.lookupTransform(target, source, tf2::TimePointZero);
}

template <typename BufferT>
auto lookupTfCompat(BufferT & buf, const std::string & target, const std::string & source, long)
  -> decltype(buf.lookupTransform(target, source, rclcpp::Time(0)))
{
  return buf.lookupTransform(target, source, rclcpp::Time(0));
}
}  // namespace

namespace rm_behavior_tree
{

RmucMicroSearchSupplyCardAction::RmucMicroSearchSupplyCardAction(
  const std::string & name,
  const BT::NodeConfig & config,
  const BT::RosNodeParams & params)
: BT::StatefulActionNode(name, config)
{
  node_ = params.nh;
  if (!node_) {
    throw BT::RuntimeError("RmucMicroSearchSupplyCard: params.nh is null");
  }

  pub_goal_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  last_pub_ms_ = 0;
  step_idx_ = 0;
  ring_idx_ = 0;
}

std::uint64_t RmucMicroSearchSupplyCardAction::nowMs_() const
{
  const auto now = node_->get_clock()->now();
  return static_cast<std::uint64_t>(now.nanoseconds() / 1000000ULL);
}

bool RmucMicroSearchSupplyCardAction::getCurrentPose_(
  geometry_msgs::msg::PoseStamped & out_pose)
{
  try {
    const auto tf = lookupTfCompat(*tf_buffer_, map_frame_, base_frame_, 0);

    out_pose.header.stamp = node_->get_clock()->now();
    out_pose.header.frame_id = map_frame_;
    out_pose.pose.position.x = tf.transform.translation.x;
    out_pose.pose.position.y = tf.transform.translation.y;
    out_pose.pose.position.z = tf.transform.translation.z;
    out_pose.pose.orientation = tf.transform.rotation;
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "RmucMicroSearchSupplyCard: TF lookup failed (%s -> %s): %s",
      map_frame_.c_str(), base_frame_.c_str(), ex.what());
    return false;
  }
}

BT::NodeStatus RmucMicroSearchSupplyCardAction::onStart()
{
  // 如果已经刷到卡，直接 SUCCESS
  bool arrived = false;
  if (getInput<bool>("rfid_supply_arrived", arrived) && arrived) {
    return BT::NodeStatus::SUCCESS;
  }
  if (!arrived) {
    auto rfid_msg = getInput<rm_decision_interfaces::msg::RMUCRFIDStatus>("rfid_status");
    if (rfid_msg && rfid_msg.value().rfid_supply) {
      arrived = true;
    }
  }
  if (arrived) {
    return BT::NodeStatus::SUCCESS;
  }

  // 初始化 search_start_ms
  std::uint64_t start_ms = 0;
  (void)getInput<std::uint64_t>("search_start_ms", start_ms);
  if (start_ms <= 0) {
    start_ms = nowMs_();
    setOutput<std::uint64_t>("search_start_ms", start_ms);
  }

  last_pub_ms_ = 0;
  step_idx_ = 0;
  ring_idx_ = 0;

  return BT::NodeStatus::RUNNING;
}

void RmucMicroSearchSupplyCardAction::publishNextGoal_(
  const geometry_msgs::msg::PoseStamped & cur)
{
  constexpr double kBaseR = 0.12;
  constexpr double kRingStep = 0.06;
  constexpr double kMaxR = 0.35;

  const double r = std::min(kMaxR, kBaseR + ring_idx_ * kRingStep);

  double dx = 0.0, dy = 0.0;
  switch (step_idx_ % 4) {
    case 0: dx = +r; dy = 0.0; break;
    case 1: dx = 0.0; dy = +r; break;
    case 2: dx = -r; dy = 0.0; break;
    default: dx = 0.0; dy = -r; break;
  }

  geometry_msgs::msg::PoseStamped goal;
  goal.header.stamp = node_->get_clock()->now();
  goal.header.frame_id = map_frame_;
  goal.pose.position.x = cur.pose.position.x + dx;
  goal.pose.position.y = cur.pose.position.y + dy;
  goal.pose.position.z = cur.pose.position.z;
  goal.pose.orientation = cur.pose.orientation;

  pub_goal_->publish(goal);
  step_idx_++;
}

BT::NodeStatus RmucMicroSearchSupplyCardAction::onRunning()
{
  // 检查刷卡
  bool arrived = false;
  if (getInput<bool>("rfid_supply_arrived", arrived) && arrived) {
    return BT::NodeStatus::SUCCESS;
  }
  if (!arrived) {
    auto rfid_msg = getInput<rm_decision_interfaces::msg::RMUCRFIDStatus>("rfid_status");
    if (rfid_msg && rfid_msg.value().rfid_supply) {
      arrived = true;
    }
  }
  if (arrived) {
    return BT::NodeStatus::SUCCESS;
  }

  // 读取 timeout 与 search_start_ms
  std::uint64_t timeout_ms = 0;
  (void)getInput<std::uint64_t>("timeout_ms", timeout_ms);

  std::uint64_t start_ms = 0;
  (void)getInput<std::uint64_t>("search_start_ms", start_ms);
  if (start_ms <= 0) {
    start_ms = nowMs_();
    setOutput<std::uint64_t>("search_start_ms", start_ms);
  }

  const std::uint64_t now_ms = nowMs_();

  // 超时扩圈
  if (timeout_ms > 0) {
    const std::uint64_t elapsed = now_ms - start_ms;
    if (elapsed >= timeout_ms) {
      ring_idx_ = std::min(ring_idx_ + 1, 6);
      setOutput<std::uint64_t>("search_start_ms", now_ms);
    }
  }

  // 节流发布：每 250ms 发一次 goal
  constexpr std::uint64_t kPubIntervalMs = 250;
  if (last_pub_ms_ != 0 && (now_ms - last_pub_ms_) < kPubIntervalMs) {
    return BT::NodeStatus::RUNNING;
  }

  geometry_msgs::msg::PoseStamped cur;
  if (!getCurrentPose_(cur)) {
    return BT::NodeStatus::RUNNING;
  }

  publishNextGoal_(cur);
  last_pub_ms_ = now_ms;

  return BT::NodeStatus::RUNNING;
}

void RmucMicroSearchSupplyCardAction::onHalted()
{
  RCLCPP_DEBUG(node_->get_logger(), "RmucMicroSearchSupplyCard halted");
}

}  // namespace rm_behavior_tree

CreateRosNodePlugin(rm_behavior_tree::RmucMicroSearchSupplyCardAction, "RmucMicroSearchSupplyCard");
