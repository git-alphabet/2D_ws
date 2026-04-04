#include "rm_behavior_tree/plugins/rmul_2026/action/clear_costmap.hpp"

#include <type_traits>

#include "behaviortree_ros2/plugins.hpp"

namespace rm_behavior_tree
{

ClearCostmapAction::ClearCostmapAction(
	const std::string & name,
	const BT::NodeConfig & conf,
	const BT::RosNodeParams & params)
: BT::SyncActionNode(name, conf)
{
	// 与 CancelNavGoalAction 相同的模式取得 node handle
	auto get_node = [](const auto & nh) -> rclcpp::Node::SharedPtr {
		using NH = std::decay_t<decltype(nh)>;
		if constexpr (std::is_convertible_v<NH, rclcpp::Node::SharedPtr>) {
			return nh;
		} else {
			return nh.lock();
		}
	};

	node_ = get_node(params.nh);
	if (!node_) {
		throw BT::RuntimeError("ClearCostmapAction: failed to obtain ROS node from params.nh");
	}
	// 注意：不在构造阶段创建 client，避免 wait_for_service 启动顺序问题
}

BT::NodeStatus ClearCostmapAction::tick()
{
	std::string service_name = "local_costmap/clear_entirely";
	getInput("service_name", service_name);

	int timeout_ms = 1000;
	getInput("timeout_ms", timeout_ms);

	// 延迟创建 service client（首次 tick 或 service 名变化时）
	if (!client_ || service_name != current_service_name_) {
		current_service_name_ = service_name;
		client_ = node_->create_client<nav2_msgs::srv::ClearEntireCostmap>(service_name);
	}

	// 非阻塞检查服务是否可用
	if (!client_->service_is_ready()) {
		RCLCPP_WARN_THROTTLE(
			node_->get_logger(), *node_->get_clock(), 5000,
			"ClearCostmap: service '%s' not ready, skipping", service_name.c_str());
		return BT::NodeStatus::SUCCESS;  // 服务不可用时不阻塞，继续执行
	}

	// 发送清空请求
	auto request = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
	auto future = client_->async_send_request(request);

	// 等待响应（有超时）
	auto status = future.wait_for(std::chrono::milliseconds(timeout_ms));
	if (status == std::future_status::ready) {
		RCLCPP_INFO(node_->get_logger(), "ClearCostmap: '%s' cleared successfully", service_name.c_str());
	} else {
		RCLCPP_WARN(node_->get_logger(), "ClearCostmap: '%s' timed out after %d ms", service_name.c_str(), timeout_ms);
	}

	return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

CreateRosNodePlugin(rm_behavior_tree::ClearCostmapAction, "ClearCostmap");
