#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__INIT_SEARCH_TIMER_IF_NEEDED_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__INIT_SEARCH_TIMER_IF_NEEDED_HPP_

#include <cstdint>
#include <string>
#include <memory>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rm_behavior_tree
{

/**
 * @brief 初始化搜卡计时起点（仅在未初始化时写入）
 *
 * 语义：
 *  - 若 search_start_ms == 0：
 *      search_start_ms <- 当前 ROS 时间（毫秒）
 *  - 否则不改
 *
 * 返回：
 *  - 成功写入或已初始化 -> SUCCESS
 *  - 读取不到端口 / node 无效 -> FAILURE（保守）
 */
class InitSearchTimerIfNeeded : public BT::SyncActionNode
{
public:
  InitSearchTimerIfNeeded(
    const std::string& name,
    const BT::NodeConfig& conf,
    const BT::RosNodeParams& params);

  static BT::PortsList providedPorts()
  {
    return {
      BT::BidirectionalPort<std::uint64_t>("search_start_ms")
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__INIT_SEARCH_TIMER_IF_NEEDED_HPP_
