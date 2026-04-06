#include "rm_behavior_tree/plugins/rmul_2026/condition/is_navigation_stuck.hpp"

#include <cmath>

#include "behaviortree_ros2/plugins.hpp"

namespace rm_behavior_tree
{

IsNavigationStuckCondition::IsNavigationStuckCondition(
	const std::string & name,
	const BT::NodeConfig & conf,
	const BT::RosNodeParams & params)
: BT::ConditionNode(name, conf), params_(params)
{
}

BT::NodeStatus IsNavigationStuckCondition::tick()
{
	auto res_x = getInput<double>("pose_x");
	auto res_y = getInput<double>("pose_y");

	if (!res_x || !res_y) {
		return BT::NodeStatus::FAILURE;
	}

	const double px = res_x.value();
	const double py = res_y.value();
	const auto now = std::chrono::steady_clock::now();

	int stuck_timeout_ms = 5000;
	int far_stuck_timeout_ms = 10000;
	double movement_threshold = 0.15;
	double reset_distance = 0.5;
	double stuck_check_radius = 1.6;
	double near_goal_skip_radius = 0.0;
	getInput("stuck_timeout_ms", stuck_timeout_ms);
	getInput("far_stuck_timeout_ms", far_stuck_timeout_ms);
	getInput("movement_threshold", movement_threshold);
	getInput("reset_distance", reset_distance);
	getInput("stuck_check_radius", stuck_check_radius);
	getInput("near_goal_skip_radius", near_goal_skip_radius);

	// 根据距离目标点的远近选择不同的超时
	// 近处（<= stuck_check_radius）：使用 stuck_timeout_ms
	// 远处（> stuck_check_radius）：使用 far_stuck_timeout_ms，0 = 不检测
	int effective_timeout_ms = stuck_timeout_ms;
	auto res_gx = getInput<double>("goal_x");
	auto res_gy = getInput<double>("goal_y");
	if (res_gx && res_gy) {
		const double dist_to_goal =
			std::hypot(px - res_gx.value(), py - res_gy.value());

		// 排除2：距目标 < near_goal_skip_radius 时不检测（已到达）
		if (near_goal_skip_radius > 0.0 && dist_to_goal < near_goal_skip_radius) {
			stuck_latched_ = false;
			initialized_ = false;
			return BT::NodeStatus::FAILURE;
		}

		if (dist_to_goal > stuck_check_radius) {
			if (far_stuck_timeout_ms <= 0) {
				// 远处不检测（保持原始行为）
				stuck_latched_ = false;
				initialized_ = false;
				return BT::NodeStatus::FAILURE;
			}
			effective_timeout_ms = far_stuck_timeout_ms;
		}
	}

	// 首次 tick：初始化参考位置和时间
	if (!initialized_) {
		last_moved_x_ = px;
		last_moved_y_ = py;
		last_move_time_ = now;
		initialized_ = true;
		return BT::NodeStatus::FAILURE;
	}

	// ConditionNode::halt() 为 final 无法重写，改用时间间隔检测陈旧状态
	// 如果距上次移动的时间远超检测超时（节点被暂停后重新启动），重新初始化
	if (!stuck_latched_) {
		const auto gap =
			std::chrono::duration_cast<std::chrono::milliseconds>(now - last_move_time_);
		if (gap.count() > effective_timeout_ms * 2) {
			last_moved_x_ = px;
			last_moved_y_ = py;
			last_move_time_ = now;
			return BT::NodeStatus::FAILURE;
		}
	}

	// 锁存中：检查是否已从卡住点移开足够距离
	if (stuck_latched_) {
		const double dist_from_stuck =
			std::hypot(px - stuck_detection_x_, py - stuck_detection_y_);
		if (dist_from_stuck > reset_distance) {
			stuck_latched_ = false;
			last_moved_x_ = px;
			last_moved_y_ = py;
			last_move_time_ = now;
			return BT::NodeStatus::FAILURE;
		}
		return BT::NodeStatus::SUCCESS;  // 仍处于卡住状态
	}

	// 检测移动
	const double dist_moved =
		std::hypot(px - last_moved_x_, py - last_moved_y_);
	if (dist_moved > movement_threshold) {
		last_moved_x_ = px;
		last_moved_y_ = py;
		last_move_time_ = now;
		return BT::NodeStatus::FAILURE;  // 正在移动，未卡住
	}

	// 未移动，检查是否超时
	const auto elapsed =
		std::chrono::duration_cast<std::chrono::milliseconds>(now - last_move_time_);
	if (elapsed.count() >= effective_timeout_ms) {
		stuck_latched_ = true;
		stuck_detection_x_ = px;
		stuck_detection_y_ = py;
		return BT::NodeStatus::SUCCESS;  // 卡住！
	}

	return BT::NodeStatus::FAILURE;  // 还没超时，继续等待
}

}  // namespace rm_behavior_tree

CreateRosNodePlugin(rm_behavior_tree::IsNavigationStuckCondition, "IsNavigationStuck");
