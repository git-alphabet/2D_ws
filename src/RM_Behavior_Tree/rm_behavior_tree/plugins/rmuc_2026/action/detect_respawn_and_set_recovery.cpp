#include "rm_behavior_tree/plugins/rmuc_2026/action/detect_respawn_and_set_recovery.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/time.hpp"

namespace rm_behavior_tree
{

RmucDetectRespawnAndSetRecoveryAction::RmucDetectRespawnAndSetRecoveryAction(
  const std::string & name,
  const BT::NodeConfig & conf,
  const BT::RosNodeParams & params)
: BT::RosTopicSubNode<sp_msgs::msg::RMUCRobotStatus>(name, conf, params),
  alive_stable_frames_(0),
  respawn_locked_(false),
  last_respawn_ms_(0),
  current_hp_(0)
{
  try {
    if (!node_) {
      RCLCPP_WARN(rclcpp::get_logger("RmucDetectRespawnAndSetRecoveryAction"),
                  "ROS node handle is null, skip fallback subscriber creation");
      return;
    }

    std::string topic = params.default_port_value.empty()
      ? std::string("robot_status") : params.default_port_value;

    if (sub_instance_) {
      fallback_signal_conn_ = sub_instance_->broadcaster.connect(
        [this](const std::shared_ptr<sp_msgs::msg::RMUCRobotStatus> msg) {
          this->fallback_last_msg_ = msg;
        });
      RCLCPP_DEBUG(node_->get_logger(),
        "Attached fallback to shared broadcaster for topic %s", topic.c_str());
    } else {
      rclcpp::SubscriptionOptions opts;
      fallback_sub_ = node_->create_subscription<sp_msgs::msg::RMUCRobotStatus>(
        topic, 10,
        [this](const std::shared_ptr<sp_msgs::msg::RMUCRobotStatus> msg) {
          this->fallback_last_msg_ = msg;
        }, opts);
      RCLCPP_DEBUG(node_->get_logger(),
        "Fallback subscriber created for topic %s", topic.c_str());
    }
  } catch (const std::bad_alloc & e) {
    RCLCPP_WARN(node_->get_logger(),
      "Memory allocation failed for fallback subscriber: %s", e.what());
  } catch (const std::exception & e) {
    RCLCPP_WARN(node_->get_logger(),
      "Failed to create fallback subscriber: %s", e.what());
  } catch (...) {
    RCLCPP_WARN(node_->get_logger(), "Unknown error creating fallback subscriber");
  }
}

RmucDetectRespawnAndSetRecoveryAction::~RmucDetectRespawnAndSetRecoveryAction()
{
  if (fallback_signal_conn_.connected()) {
    fallback_signal_conn_.disconnect();
  }
  fallback_sub_.reset();
  fallback_last_msg_.reset();
}

BT::NodeStatus RmucDetectRespawnAndSetRecoveryAction::onTick(
  const std::shared_ptr<sp_msgs::msg::RMUCRobotStatus> & last_msg)
{
  try {
    if (!fallback_signal_conn_.connected() && sub_instance_ && node_) {
      fallback_signal_conn_ = sub_instance_->broadcaster.connect(
        [this](const std::shared_ptr<sp_msgs::msg::RMUCRobotStatus> msg) {
          this->fallback_last_msg_ = msg;
        });
      RCLCPP_DEBUG(node_->get_logger(),
        "Deferred attach of fallback to shared broadcaster");
    }
  } catch (...) {}

  if (!node_) {
    RCLCPP_WARN(rclcpp::get_logger("RmucDetectRespawnAndSetRecoveryAction"),
                "Node handle is null, skip respawn detection");
    return BT::NodeStatus::SUCCESS;
  }

  // 1. 读取血量（消息优先，黑板兜底；并写回黑板）
  bool got_hp = false;
  current_hp_ = 0;

  if (last_msg) {
    current_hp_ = static_cast<int>(last_msg->current_hp);
    got_hp = true;
    RCLCPP_DEBUG(node_->get_logger(),
      "Debug: last_msg present, current_hp=%d", current_hp_);
  } else if (fallback_last_msg_) {
    current_hp_ = static_cast<int>(fallback_last_msg_->current_hp);
    got_hp = true;
    RCLCPP_DEBUG(node_->get_logger(),
      "Debug: using fallback_last_msg, current_hp=%d", current_hp_);
  } else if (auto res = getInput<int>("hp_cur")) {
    current_hp_ = res.value();
    got_hp = true;
    RCLCPP_DEBUG(node_->get_logger(),
      "Debug: using hp_cur from blackboard, current_hp=%d", current_hp_);
  }

  if (!got_hp) {
    RCLCPP_WARN(node_->get_logger(), "未获取到机器人血量，跳过复活沿检测");
    return BT::NodeStatus::SUCCESS;
  }

  setOutput("hp_cur", current_hp_);

  // 2. 读取历史死亡状态
  bool was_dead = false;
  if (auto res = getInput<bool>("was_dead")) {
    was_dead = res.value();
  }

  // 3. 读取当前恢复标志
  bool need_recovery = false;
  if (auto res = getInput<bool>("need_recovery")) {
    need_recovery = res.value();
  }

  // 4. 当前时间（毫秒）
  std::uint64_t now_ms =
    static_cast<std::uint64_t>(node_->now().nanoseconds() / 1000000ULL);

  RCLCPP_DEBUG(node_->get_logger(),
    "[DetectRespawn] hp=%d, was_dead=%d, need_recovery=%d, alive_frames=%d/%d",
    current_hp_, static_cast<int>(was_dead), static_cast<int>(need_recovery),
    alive_stable_frames_, RESPAWN_STABLE_FRAMES);

  // 5. 血量合法性过滤
  const bool hp_is_valid = (current_hp_ >= 0 && current_hp_ <= MAX_HP);
  const bool is_dead_now = hp_is_valid ? (current_hp_ == 0) : true;

  // 6. 连续帧防抖
  if (!is_dead_now && hp_is_valid) {
    alive_stable_frames_ = std::min(alive_stable_frames_ + 1, RESPAWN_STABLE_FRAMES);
  } else {
    alive_stable_frames_ = 0;
  }

  // 7. 复活触发锁
  bool lock_expired = false;
  if (last_respawn_ms_ > now_ms) {
    lock_expired = true;
  } else {
    lock_expired = (now_ms - last_respawn_ms_ > RESPAWN_LOCK_TIMEOUT);
  }

  if (respawn_locked_ && !lock_expired) {
    respawn_locked_ = true;
  } else {
    respawn_locked_ = false;
  }

  // 8. 最终复活沿判断
  const bool respawn_edge = (was_dead &&
                            (alive_stable_frames_ >= RESPAWN_STABLE_FRAMES) &&
                            current_hp_ > 0 &&
                            hp_is_valid &&
                            !respawn_locked_);

  // 9. 触发复活沿：初始化恢复参数
  if (respawn_edge) {
    need_recovery = true;
    setOutput("recovery_start_ms", now_ms);
    setOutput("search_start_ms", static_cast<std::uint64_t>(0));
    setOutput("heal_start_ms", static_cast<std::uint64_t>(0));

    respawn_locked_ = true;
    last_respawn_ms_ = now_ms;

    RCLCPP_INFO(node_->get_logger(),
          "检测到机器人复活沿！当前血量：%d，触发恢复模式", current_hp_);
    RCLCPP_DEBUG(node_->get_logger(),
      "Debug: set outputs: was_dead=%d need_recovery=%d recovery_start_ms=%llu",
      static_cast<int>(is_dead_now), static_cast<int>(need_recovery),
      static_cast<unsigned long long>(now_ms));
  }

  // 10. 更新黑板状态
  if (is_dead_now) {
    setOutput("was_dead", true);
  } else if (respawn_edge) {
    setOutput("was_dead", false);
  } else {
    setOutput("was_dead", was_dead);
  }
  setOutput("need_recovery", need_recovery);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::RmucDetectRespawnAndSetRecoveryAction, "RmucDetectRespawnAndSetRecovery");
