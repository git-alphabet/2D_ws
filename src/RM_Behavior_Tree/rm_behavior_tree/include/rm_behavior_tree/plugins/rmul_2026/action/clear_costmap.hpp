#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__CLEAR_COSTMAP_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__CLEAR_COSTMAP_HPP_

#include <string>
#include <chrono>
#include <memory>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"

namespace rm_behavior_tree
{

/**
 * @brief 清空 Nav2 costmap 的 BT 动作节点
 *
 * 使用延迟初始化避免 Nav2 启动顺序问题：
 * - 不在构造阶段 wait_for_service（这是导致 ClearEntireCostmap 被移除的根因）
 * - 在首次 tick() 时才创建 service client
 * - 如果服务不可用，不阻塞，返回 SUCCESS 继续执行
 */
class ClearCostmapAction : public BT::SyncActionNode
{
public:
	ClearCostmapAction(
		const std::string & name,
		const BT::NodeConfig & conf,
		const BT::RosNodeParams & params);

	static BT::PortsList providedPorts()
	{
		return {
			BT::InputPort<std::string>("service_name",
				"local_costmap/clear_entirely",
				"ClearEntireCostmap 服务名"),
			BT::InputPort<int>("timeout_ms", "1000",
				"等待服务响应超时 (ms)")
		};
	}

	BT::NodeStatus tick() override;

private:
	rclcpp::Node::SharedPtr node_;
	rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr client_;
	std::string current_service_name_;
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__CLEAR_COSTMAP_HPP_
