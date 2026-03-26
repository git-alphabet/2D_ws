#ifndef RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_NAVIGATION_STUCK_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_NAVIGATION_STUCK_HPP_

#include <string>
#include <chrono>

#include "behaviortree_cpp/condition_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"

namespace rm_behavior_tree
{

/**
 * @brief 导航卡住检测条件节点
 *
 * 监测机器人位置变化，若在 stuck_timeout_ms 内移动距离未超过
 * movement_threshold，则判定为卡住（返回 SUCCESS）。
 *
 * 采用"锁存+自动解锁"策略：
 *   - 一旦判定卡住，持续返回 SUCCESS（锁存），直到机器人从卡住点
 *     移动超过 reset_distance（例如导航到偏移点后解锁）。
 *   - 解锁后重新尝试正常导航，若再次卡住则重新锁存。
 *   - 子树退出时通过 halt() 重置所有状态。
 */
class IsNavigationStuckCondition : public BT::ConditionNode
{
public:
	IsNavigationStuckCondition(
		const std::string & name,
		const BT::NodeConfig & conf,
		const BT::RosNodeParams & params);

	static BT::PortsList providedPorts()
	{
		return {
			BT::InputPort<double>("pose_x"),
			BT::InputPort<double>("pose_y"),
			BT::InputPort<double>("goal_x"),
			BT::InputPort<double>("goal_y"),
			BT::InputPort<double>("stuck_check_radius", "1.6",
				"距目标点此范围内使用 stuck_timeout_ms；超出则使用 far_stuck_timeout_ms"),
			BT::InputPort<int>("stuck_timeout_ms", "5000",
				"进入 stuck_check_radius 后判定卡住的超时 (ms)"),
			BT::InputPort<int>("far_stuck_timeout_ms", "10000",
				"远离目标点时判定卡住的超时 (ms)，0=远处不检测"),
			BT::InputPort<double>("movement_threshold", "0.15",
				"认为发生移动的最小距离 (m)"),
			BT::InputPort<double>("reset_distance", "0.5",
				"从卡住点移动此距离后解除锁存 (m)"),
			BT::InputPort<double>("near_goal_skip_radius", "0.0",
				"距目标点 < 此距离时不判卡住—已到达 (m)，0=禁用")
		};
	}

	BT::NodeStatus tick() override;

private:
	BT::RosNodeParams params_;

	// 位置追踪
	double last_moved_x_{0.0};
	double last_moved_y_{0.0};
	std::chrono::steady_clock::time_point last_move_time_;
	bool initialized_{false};

	// 锁存状态
	bool stuck_latched_{false};
	double stuck_detection_x_{0.0};
	double stuck_detection_y_{0.0};
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_NAVIGATION_STUCK_HPP_
