#include "rm_behavior_tree/plugins/rmul_2026/condition/is_robot_stuck.hpp"

#include <cmath>

#include "behaviortree_ros2/plugins.hpp"

namespace rm_behavior_tree
{

IsRobotStuckCondition::IsRobotStuckCondition(
	const std::string & name,
	const BT::NodeConfig & conf,
	const BT::RosNodeParams & params)
: BT::ConditionNode(name, conf), params_(params)
{
}

BT::NodeStatus IsRobotStuckCondition::tick()
{
	auto res_x = getInput<double>("pose_x");
	auto res_y = getInput<double>("pose_y");

	if (!res_x || !res_y) {
		return BT::NodeStatus::FAILURE;
	}

	const double px = res_x.value();
	const double py = res_y.value();
	const auto now = std::chrono::steady_clock::now();

	int stuck_timeout_ms = 8000;
	double movement_threshold = 0.15;
	double reset_distance = 0.5;
	getInput("stuck_timeout_ms", stuck_timeout_ms);
	getInput("movement_threshold", movement_threshold);
	getInput("reset_distance", reset_distance);

	// 首次 tick：初始化参考位置和时间
	if (!initialized_) {
		last_moved_x_ = px;
		last_moved_y_ = py;
		last_move_time_ = now;
		initialized_ = true;
		return BT::NodeStatus::FAILURE;
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
	if (elapsed.count() >= stuck_timeout_ms) {
		stuck_latched_ = true;
		stuck_detection_x_ = px;
		stuck_detection_y_ = py;
		return BT::NodeStatus::SUCCESS;  // 卡住！
	}

	return BT::NodeStatus::FAILURE;  // 还没超时，继续等待
}

void IsRobotStuckCondition::halt()
{
	initialized_ = false;
	stuck_latched_ = false;
	BT::ConditionNode::halt();
}

}  // namespace rm_behavior_tree

CreateRosNodePlugin(rm_behavior_tree::IsRobotStuckCondition, "IsRobotStuck");
