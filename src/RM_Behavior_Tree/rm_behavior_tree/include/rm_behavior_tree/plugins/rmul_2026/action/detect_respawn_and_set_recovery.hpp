#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__DETECT_RESPAWN_AND_SET_RECOVERY_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__DETECT_RESPAWN_AND_SET_RECOVERY_HPP_

#include <cstdint>
#include <string>
#include <algorithm>  // 用于std::min
#include <limits>     // 用于数值溢出保护

#include <boost/signals2/connection.hpp>
#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "rm_decision_interfaces/msg/rmul.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rm_behavior_tree
{

class DetectRespawnAndSetRecoveryAction
: public BT::RosTopicSubNode<rm_decision_interfaces::msg::RMUL>
{
public:
  // 构造函数（保持原有）
  DetectRespawnAndSetRecoveryAction(
    const std::string & name,
    const BT::NodeConfig & conf,
    const BT::RosNodeParams & params);

  // 新增析构函数：释放资源
  ~DetectRespawnAndSetRecoveryAction() override;

  // 修正语法错误：移除嵌套return，正确复用providedBasicPorts
  static BT::PortsList providedPorts()
  {
    // 核心修正：直接return providedBasicPorts的返回值，而非嵌套return
    return providedBasicPorts({
      BT::BidirectionalPort<int>("hp_cur"),
      BT::BidirectionalPort<bool>("was_dead"),
      BT::BidirectionalPort<bool>("need_recovery"),
      BT::BidirectionalPort<std::uint64_t>("recovery_start_ms"),
      BT::BidirectionalPort<std::uint64_t>("search_start_ms"),
      BT::BidirectionalPort<std::uint64_t>("heal_start_ms")
    });
  }

  BT::NodeStatus onTick(
    const std::shared_ptr<rm_decision_interfaces::msg::RMUL> & last_msg) override;

private:
  // 1. 复活防抖配置：连续N帧存活才判定真复活（10Hz下2帧=200ms，适配竞赛）
  static constexpr int RESPAWN_STABLE_FRAMES = 2;
  // 2. 存活状态连续帧数缓存（防抖核心）
  int alive_stable_frames_ = 0;
  // 3. 复活触发锁：避免短时间重复触发
  bool respawn_locked_ = false;
  // 4. 锁超时时间（ms）：触发复活后5秒内不重复检测
  static constexpr std::uint64_t RESPAWN_LOCK_TIMEOUT = 5000;
  // 5. 上次触发复活的时间戳（用于锁超时判断）
  std::uint64_t last_respawn_ms_ = 0;
  // 6. 当前血量（成员变量，避免局部变量未定义问题）
  int current_hp_ = 0;
  // 7. 恢复模式退出阈值（从参数服务器读取）
  int recovery_exit_hp_ = 400;
  // Fallback subscription in case the RosTopicSubNode registry-based subscriber
  // does not deliver messages to this instance (diagnostic fallback).
  rclcpp::Subscription<rm_decision_interfaces::msg::RMUL>::SharedPtr fallback_sub_;
  std::shared_ptr<rm_decision_interfaces::msg::RMUL> fallback_last_msg_ = nullptr;
  // connection to the shared SubscriberInstance broadcaster (diagnostic fallback)
  boost::signals2::connection fallback_signal_conn_;
  // 8. 机器人最大血量
  static constexpr int MAX_HP = 400;
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__DETECT_RESPAWN_AND_SET_RECOVERY_HPP_
