#include "rm_behavior_tree/plugins/rmul_2026/action/micro_search_supply_card.hpp"
#include "behaviortree_ros2/plugins.hpp"

#include <iostream>

// --------- TF lookup 的“签名自适配”小工具 ---------
// 有的 tf2_ros::Buffer 版本用 tf2::TimePointZero
// 有的版本用 rclcpp::Time(0)
// 用 SFINAE 让编译器自动选能用的那个
namespace
{
template <typename BufferT>
auto lookupTfCompat(BufferT& buf, const std::string& target, const std::string& source, int)
  -> decltype(buf.lookupTransform(target, source, tf2::TimePointZero))
{
  return buf.lookupTransform(target, source, tf2::TimePointZero);
}

template <typename BufferT>
auto lookupTfCompat(BufferT& buf, const std::string& target, const std::string& source, long)
  -> decltype(buf.lookupTransform(target, source, rclcpp::Time(0)))
{
  return buf.lookupTransform(target, source, rclcpp::Time(0));
}
} // namespace

namespace rm_behavior_tree
{

MicroSearchSupplyCardAction::MicroSearchSupplyCardAction(
  const std::string& name,
  const BT::NodeConfig& config,
  const BT::RosNodeParams& params)
: BT::StatefulActionNode(name, config)
{
  // ✅ 你们版本 nh 是 shared_ptr，不要 lock()
  node_ = params.nh;
  if (!node_) {
    throw BT::RuntimeError("MicroSearchSupplyCard: params.nh is null");
  }

  // 话题名写死：goal_pose
  pub_goal_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  last_pub_ms_ = 0;
  step_idx_ = 0;
  ring_idx_ = 0;
}

std::uint64_t MicroSearchSupplyCardAction::nowMs_() const
{
  const auto now = node_->get_clock()->now();
  return static_cast<std::uint64_t>(now.nanoseconds() / 1000000ULL);
}

bool MicroSearchSupplyCardAction::getCurrentPose_(geometry_msgs::msg::PoseStamped& out_pose)
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
  }
  catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "MicroSearchSupplyCard: TF lookup failed (%s -> %s): %s",
      map_frame_.c_str(), base_frame_.c_str(), ex.what());
    return false;
  }
}

BT::NodeStatus MicroSearchSupplyCardAction::onStart()
{
  // 如果已经刷到卡，直接 SUCCESS（外层 ReactiveFallback 也会抢占，这里只是更干净）
  bool arrived = false;
  if (getInput<bool>("rfid_supply_arrived", arrived) && arrived) {
    return BT::NodeStatus::SUCCESS;
  }
  if (!arrived) {
    auto rfid_msg = getInput<rm_decision_interfaces::msg::RMUL>("rfid_status");
    if (rfid_msg && rfid_msg.value().rfid_supply_arrived) {
      arrived = true;
    }
  }
  if (arrived) {
    return BT::NodeStatus::SUCCESS;
  }

  // 初始化 search_start_ms（首次进入写入）
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

void MicroSearchSupplyCardAction::publishNextGoal_(const geometry_msgs::msg::PoseStamped& cur)
{
  // 微搜半径策略：小十字四点 + 超时扩圈
  constexpr double kBaseR = 0.12;     // 12cm
  constexpr double kRingStep = 0.06;  // 每次超时扩 6cm
  constexpr double kMaxR = 0.35;      // 最大 35cm，避免跑出补给区

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

  // 保持朝向，避免控制器为追朝向出现奇怪旋转
  goal.pose.orientation = cur.pose.orientation;

  pub_goal_->publish(goal);
  step_idx_++;
}

BT::NodeStatus MicroSearchSupplyCardAction::onRunning()
{
  // ✅ 只用 bool 判断刷卡
  bool arrived = false;
  if (getInput<bool>("rfid_supply_arrived", arrived) && arrived) {
    return BT::NodeStatus::SUCCESS;
  }
  if (!arrived) {
    auto rfid_msg = getInput<rm_decision_interfaces::msg::RMUL>("rfid_status");
    if (rfid_msg && rfid_msg.value().rfid_supply_arrived) {
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

  // 超时：扩圈 + 重置计时（不返回 FAILURE，避免回到导航分支抖动）
  if (timeout_ms > 0) {
    const std::uint64_t elapsed = now_ms - start_ms;
    if (elapsed >= timeout_ms) {
      ring_idx_ = std::min(ring_idx_ + 1, 6);
      setOutput<std::uint64_t>("search_start_ms", now_ms);
      // 可选：重置步进，让扩圈从第一个点开始
      // step_idx_ = 0;
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

void MicroSearchSupplyCardAction::onHalted()
{
  // 不做阻塞操作；刹车/停转由你们树里的 RobotControl/StopMotion 负责
  RCLCPP_DEBUG(node_->get_logger(), "MicroSearchSupplyCard halted");
}

}  // namespace rm_behavior_tree

CreateRosNodePlugin(rm_behavior_tree::MicroSearchSupplyCardAction, "MicroSearchSupplyCard");
