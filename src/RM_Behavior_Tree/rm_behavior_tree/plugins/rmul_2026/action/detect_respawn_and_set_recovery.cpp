#include "rm_behavior_tree/plugins/rmul_2026/action/detect_respawn_and_set_recovery.hpp"
#include "rclcpp/logging.hpp"  // 用于日志输出
#include "rclcpp/time.hpp"     // 用于时间处理

namespace rm_behavior_tree
{
 
DetectRespawnAndSetRecoveryAction::DetectRespawnAndSetRecoveryAction(
  const std::string & name,
  const BT::NodeConfig & conf,
  const BT::RosNodeParams & params)
: BT::RosTopicSubNode<rm_decision_interfaces::msg::RMUL>(name, conf, params),
  alive_stable_frames_(0),
  respawn_locked_(false),
  last_respawn_ms_(0),
  current_hp_(0)
{
  // Create a lightweight fallback subscription on the same node to ensure
  // we can observe messages even if the registry-based subscriber isn't
  // delivering into this instance for some reason. This subscription uses
  // the params default topic name.
  try {
    // 新增：检查node_是否为空，避免空指针
    if (!node_) {
      RCLCPP_WARN(rclcpp::get_logger("DetectRespawnAndSetRecoveryAction"), 
                  "ROS node handle is null, skip fallback subscriber creation");
      return;
    }

    if (!node_->has_parameter("supply_heal_min_hp")) {
      node_->declare_parameter("supply_heal_min_hp", MAX_HP);
    }
    if (!node_->has_parameter("hp_medium_threshold")) {
      node_->declare_parameter("hp_medium_threshold", 100);
    }
    node_->get_parameter("supply_heal_min_hp", recovery_exit_hp_);
    node_->get_parameter("hp_medium_threshold", low_hp_trigger_);
    recovery_exit_hp_ = std::clamp(recovery_exit_hp_, 0, MAX_HP);
    low_hp_trigger_ = std::clamp(low_hp_trigger_, 0, MAX_HP);

    std::string topic = params.default_port_value.empty() ? std::string("robot_status") : params.default_port_value;
    
    // If the registry-based SubscriberInstance already exists, attach to its
    // broadcaster so we get callbacks via the same executor used by the
    // registry (no need to create a separate subscription that requires
    // spinning the node's executor).
    if (sub_instance_) {
      // connect to the shared broadcaster
      fallback_signal_conn_ = sub_instance_->broadcaster.connect(
        [this](const std::shared_ptr<rm_decision_interfaces::msg::RMUL> msg) {
          this->fallback_last_msg_ = msg;
        });
      RCLCPP_DEBUG(node_->get_logger(), "Attached fallback to shared broadcaster for topic %s", topic.c_str());
    } else {
      // fallback to a standalone subscription (may require the node to be spun)
      rclcpp::SubscriptionOptions opts;
      fallback_sub_ = node_->create_subscription<rm_decision_interfaces::msg::RMUL>(
        topic, 10, [this](const std::shared_ptr<rm_decision_interfaces::msg::RMUL> msg) {
          this->fallback_last_msg_ = msg;
        }, opts);
      RCLCPP_DEBUG(node_->get_logger(), "Fallback subscriber created for topic %s", topic.c_str());
    }
  } catch (const std::bad_alloc& e) {
    // 补充捕获内存分配异常
    RCLCPP_WARN(node_->get_logger(), "Memory allocation failed for fallback subscriber: %s", e.what());
  } catch (const std::exception & e) {
    RCLCPP_WARN(node_->get_logger(), "Failed to create fallback subscriber: %s", e.what());
  } catch (...) {
    // 捕获未知异常，避免程序崩溃
    RCLCPP_WARN(node_->get_logger(), "Unknown error creating fallback subscriber");
  }
}

// 新增析构函数：释放信号连接和订阅器资源
DetectRespawnAndSetRecoveryAction::~DetectRespawnAndSetRecoveryAction()
{
  // 断开信号连接
  if (fallback_signal_conn_.connected()) {
    fallback_signal_conn_.disconnect();
  }
  // 释放订阅器（ROS2智能指针自动管理，显式置空更安全）
  fallback_sub_.reset();
  fallback_last_msg_.reset();
}

BT::NodeStatus DetectRespawnAndSetRecoveryAction::onTick(
  const std::shared_ptr<rm_decision_interfaces::msg::RMUL> & last_msg)
{
  // If the shared SubscriberInstance becomes available after construction,
  // attach our fallback listener to its broadcaster so we reliably receive
  // messages via the same executor (no need to spin the node's executor).
  try {
    if (!fallback_signal_conn_.connected() && sub_instance_ && node_) {
      fallback_signal_conn_ = sub_instance_->broadcaster.connect(
        [this](const std::shared_ptr<rm_decision_interfaces::msg::RMUL> msg) {
          this->fallback_last_msg_ = msg;
        });
      RCLCPP_DEBUG(node_->get_logger(), "Deferred attach of fallback to shared broadcaster");
    }
  } catch (...) {
    // ignore
  }

  // 新增：检查node_是否为空，避免后续崩溃
  if (!node_) {
    RCLCPP_WARN(rclcpp::get_logger("DetectRespawnAndSetRecoveryAction"), 
                "Node handle is null, skip respawn detection");
    return BT::NodeStatus::SUCCESS;
  }

  // 1. 读取血量（消息优先，黑板兜底；并写回黑板）
  bool got_hp = false;
  current_hp_ = 0;  // 重置默认值

  if (last_msg) {
    current_hp_ = static_cast<int>(last_msg->current_hp);
    got_hp = true;
    RCLCPP_DEBUG(node_->get_logger(), "Debug: last_msg present, current_hp=%d", current_hp_);
  }
  else if (fallback_last_msg_) {
    current_hp_ = static_cast<int>(fallback_last_msg_->current_hp);
    got_hp = true;
    RCLCPP_DEBUG(node_->get_logger(), "Debug: using fallback_last_msg, current_hp=%d", current_hp_);
  }
  else if (auto res = getInput<int>("hp_cur")) {
    current_hp_ = res.value();
    got_hp = true;
    RCLCPP_DEBUG(node_->get_logger(), "Debug: using hp_cur from blackboard, current_hp=%d", current_hp_);
  }

  if (!got_hp) {
    bool was_dead_cached = false;
    bool need_recovery_cached = false;
    if (auto res = getInput<bool>("was_dead")) {
      was_dead_cached = res.value();
    }
    if (auto res = getInput<bool>("need_recovery")) {
      need_recovery_cached = res.value();
    }

    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000,
      "未获取到机器人血量，跳过复活沿检测（保持现有恢复状态）");

    // 保持已有状态，避免输入瞬时丢失导致恢复状态被误清。
    setOutput("need_recovery", need_recovery_cached);
    setOutput("was_dead", was_dead_cached);
    return BT::NodeStatus::SUCCESS;
  }

  // 写回黑板：兑现 BidirectionalPort 的语义
  setOutput("hp_cur", current_hp_);

  // 2. 读取历史死亡状态（was_dead）
  bool was_dead = false;
  if (auto res = getInput<bool>("was_dead")) {
    was_dead = res.value();
  }

  // 3. 读取当前恢复标志
  bool need_recovery = false;
  if (auto res = getInput<bool>("need_recovery")) {
    need_recovery = res.value();
  }

  // 4. 直接从ROS节点获取当前时间（毫秒级）+ 空指针保护
  std::uint64_t now_ms = static_cast<std::uint64_t>(node_->now().nanoseconds() / 1000000ULL);

  // 5. 优化1：血量合法性过滤（避免电控解析异常值）
  const bool hp_is_valid = (current_hp_ >= 0 && current_hp_ <= MAX_HP);
  // 只有血量合法时，才判断死亡状态；非法则默认判定为死亡
  const bool is_dead_now = hp_is_valid ? (current_hp_ == 0) : true;

  // 6. 优化2：连续帧防抖（核心抗干扰逻辑）
  if (!is_dead_now && hp_is_valid) {
    // 当前存活且血量合法：累计稳定帧数（最多到配置的帧数）
    alive_stable_frames_ = std::min(alive_stable_frames_ + 1, RESPAWN_STABLE_FRAMES);
  } else {
    // 当前死亡/血量异常：重置稳定帧数
    alive_stable_frames_ = 0;
  }

  // 7. 优化3：复活触发锁（避免短时间重复触发）+ 溢出保护
  bool lock_expired = false;
  // 核心修正：uint64_t相减溢出保护（若last_respawn_ms_ > now_ms，直接判定为过期）
  if (last_respawn_ms_ > now_ms) {
    lock_expired = true;
  } else {
    lock_expired = (now_ms - last_respawn_ms_ > RESPAWN_LOCK_TIMEOUT);
  }

  if (respawn_locked_ && !lock_expired) {
    // 锁未过期：保持锁定状态，跳过检测
    respawn_locked_ = true;
  } else {
    // 锁过期/未锁定：解锁
    respawn_locked_ = false;
  }

  // 8. 最终复活沿判断（竞赛级抗干扰）
  // 条件：历史死亡 + 连续N帧存活 + 血量合法且>0 + 未被锁定
  const bool respawn_edge = (was_dead && 
                            (alive_stable_frames_ >= RESPAWN_STABLE_FRAMES) && 
                            current_hp_ > 0 && 
                            hp_is_valid && 
                            !respawn_locked_);

  // 如果当前血量较低，且存活，且未在恢复模式，强制进入恢复模式
  // 低血阈值来自参数 hp_medium_threshold
  const bool low_hp_edge = (!need_recovery && 
                            current_hp_ > 0 && 
                            current_hp_ < low_hp_trigger_ && 
                            hp_is_valid && 
                            !respawn_locked_);

  if (need_recovery && hp_is_valid && current_hp_ >= recovery_exit_hp_) {
    need_recovery = false;
    setOutput("need_recovery", false);
    setOutput("heal_start_ms", static_cast<std::uint64_t>(0));
    setOutput("search_start_ms", static_cast<std::uint64_t>(0));
    setOutput("recovery_start_ms", static_cast<std::uint64_t>(0));
    RCLCPP_INFO(node_->get_logger(),
      "恢复模式自动退出：当前血量=%d 达到阈值=%d", current_hp_, recovery_exit_hp_);
  }

  // 临时开启 INFO 以排查低血量问题 (每隔两秒打印一次以防刷屏)
  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
    "[DetectRespawn] hp=%d, was_dead=%d, need_recovery=%d, is_dead_now=%d, low_hp_edge=%d (valid=%d, lock=%d)",
    current_hp_, static_cast<int>(was_dead), static_cast<int>(need_recovery), 
    static_cast<int>(is_dead_now), 
    static_cast<int>(low_hp_edge),
    static_cast<int>(hp_is_valid), static_cast<int>(respawn_locked_));

  // 9. 触发复活沿：初始化恢复参数 + 加锁防重复
  if (respawn_edge || low_hp_edge) {
    need_recovery = true;
    setOutput("need_recovery", true); // 立即写回黑板
    setOutput("recovery_start_ms", now_ms);                       // 记录恢复开始时间
    setOutput("search_start_ms", static_cast<std::uint64_t>(0));  // 重置搜卡时间
    setOutput("heal_start_ms", static_cast<std::uint64_t>(0));    // 重置回血时间
    
    // 加锁+记录触发时间（避免短时间重复触发）
    respawn_locked_ = true;
    last_respawn_ms_ = now_ms;

    // 竞赛调试：打印复活日志（保留INFO级别，关键事件）
    RCLCPP_INFO(node_->get_logger(), 
          "检测到机器人复活沿！当前血量：%d，触发恢复模式", current_hp_);
    // 核心修正：调试日志改为DEBUG级别，避免刷屏
    RCLCPP_DEBUG(node_->get_logger(), "Debug: set outputs: was_dead=%d need_recovery=%d recovery_start_ms=%llu", 
                 static_cast<int>(is_dead_now), static_cast<int>(need_recovery), 
                 static_cast<unsigned long long>(now_ms));
  }

  // 10. 更新黑板状态（关键：同步最新状态）
  // was_dead 的语义：标记"最近经历过死亡，尚未完成复活沿检测"
  // - 死亡时 (is_dead_now=true): 设为 true
  // - 触发复活沿后 (respawn_edge=true): 设为 false（任务完成）
  // - 存活但尚在累计防抖帧 (was_dead=true, !respawn_edge): 保持 true
  if (is_dead_now) {
    setOutput("was_dead", true);
  } else if (respawn_edge) {
    setOutput("was_dead", false);               // 复活沿已触发，清除标记
  } else {
    setOutput("was_dead", was_dead);            // 保持原值（防抖期间不清除）
  }
  setOutput("need_recovery", need_recovery);   // 更新恢复模式标志

  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::DetectRespawnAndSetRecoveryAction, "DetectRespawnAndSetRecovery");
