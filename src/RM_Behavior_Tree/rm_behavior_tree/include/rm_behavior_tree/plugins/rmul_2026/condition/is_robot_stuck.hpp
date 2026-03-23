#ifndef RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_ROBOT_STUCK_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_ROBOT_STUCK_HPP_

#include <string>
#include <chrono>

#include "behaviortree_cpp/condition_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"

namespace rm_behavior_tree
{

/**
 * @brief 通用的机器人卡住检测（不依赖距离目标点远近）
 *
 * 与 IsNavigationStuck 不同：此节点在任何位置都生效，
 * 适用于比赛开场机器人拥挤、雷达被遮挡等导致导航整体失效的场景。
 *
 * 判定逻辑：
 *   - 在 stuck_timeout_ms 内移动距离 < movement_threshold → 判定卡住 (SUCCESS)
 *   - 锁存策略：一旦卡住，持续返回 SUCCESS，直到移动 > reset_distance
 */
class IsRobotStuckCondition : public BT::ConditionNode
{
public:
	IsRobotStuckCondition(
		const std::string & name,
		const BT::NodeConfig & conf,
		const BT::RosNodeParams & params);

	static BT::PortsList providedPorts()
	{
		return {
			BT::InputPort<double>("pose_x"),
			BT::InputPort<double>("pose_y"),
			BT::InputPort<int>("stuck_timeout_ms", "8000",
				"判定卡住的超时时间 (ms)"),
			BT::InputPort<double>("movement_threshold", "0.15",
				"认为发生移动的最小距离 (m)"),
			BT::InputPort<double>("reset_distance", "0.5",
				"从卡住点移动此距离后解除锁存 (m)")
		};
	}

	BT::NodeStatus tick() override;

private:
	BT::RosNodeParams params_;

	double last_moved_x_{0.0};
	double last_moved_y_{0.0};
	std::chrono::steady_clock::time_point last_move_time_;
	bool initialized_{false};

	bool stuck_latched_{false};
	double stuck_detection_x_{0.0};
	double stuck_detection_y_{0.0};
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_ROBOT_STUCK_HPP_
