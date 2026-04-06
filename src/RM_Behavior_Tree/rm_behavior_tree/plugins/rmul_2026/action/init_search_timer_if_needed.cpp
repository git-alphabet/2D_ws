#include "rm_behavior_tree/plugins/rmul_2026/action/init_search_timer_if_needed.hpp"

namespace rm_behavior_tree
{

InitSearchTimerIfNeeded::InitSearchTimerIfNeeded(
  const std::string& name,
  const BT::NodeConfig& conf,
  const BT::RosNodeParams& params)
: BT::SyncActionNode(name, conf),
  node_(params.nh)   // behaviortree_ros2 里常用字段名就是 nh（Node Handle）
{
}

BT::NodeStatus InitSearchTimerIfNeeded::tick()
{
  // 1) 读取 inout: search_start_ms
  std::uint64_t search_start_ms = 0ULL;
  if (!getInput<std::uint64_t>("search_start_ms", search_start_ms)) {
    return BT::NodeStatus::FAILURE;
  }

  // 已初始化则直接成功
  if (search_start_ms != 0ULL) {
    return BT::NodeStatus::SUCCESS;
  }

  // 2) 获取 ROS 时间
  if (!node_) {
    return BT::NodeStatus::FAILURE;
  }

  // node_->now() 为 rclcpp::Time（纳秒），转换为毫秒
  const std::uint64_t now_ms =
    static_cast<std::uint64_t>(node_->now().nanoseconds() / 1000000ULL);

  if (now_ms == 0ULL) {
    // 极端情况：时间不可用（例如 clock 未起），保守失败
    return BT::NodeStatus::FAILURE;
  }

  // 3) 写回 search_start_ms
  setOutput("search_start_ms", now_ms);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::InitSearchTimerIfNeeded, "InitSearchTimerIfNeeded");
